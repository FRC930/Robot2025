package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A command composition that runs one of any number of commands, depending on the value of the given
 * condition when this command is initialized.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 * 
 * @param <K> the type of key used
 */
public class MultiConditionalCommand<K> extends Command {
    private HashMap<K, Command> m_conditionMap;
    private Command m_selectedCommand;
    private Supplier<K> m_conditionSupplier;
    
    // Needed to avoid local variable error
    private boolean m_runsWhenDisabled;
    private InterruptionBehavior m_interruptionBehavior;

    public static class ConditionCommandEntry<K> implements Map.Entry<K, Command>{

        final K condition;
        Command command;

        public ConditionCommandEntry(K condition, Command command) {
            this.condition = condition;
            this.command = command;
        }

        @Override
        public K getKey() {
            return condition;
        }

        @Override
        public Command getValue() {
            return command;
        }

        @Override
        public Command setValue(Command command) {
            Command oldCommand = this.command;
            this.command = command;
            return oldCommand;
        }

    }

    /**
     * Creates a new MultiConditionalCommand
     * 
     * @param defaultCommand the command to run if the value of condition does not match any conditions in map
     * @param condition the condition to determine which command to run
     * @param conditionValues alternating keys and values to be added to conditions mapping. Must have an even number of objects for key-value pairs. 
     * Every second object must be a Command. First object of each set of two objects is the condition to be mapped to the command following it.
     */
    public MultiConditionalCommand(Command defaultCommand, Supplier<K> condition, Object... conditionValues) {
        m_selectedCommand = defaultCommand;

        m_conditionMap = new HashMap<K, Command>();

        if (conditionValues.length % 2 != 0) {
            throw new IllegalArgumentException("The number of arguments must be even (key-value pairs).");
        }

        for (int i = 0; i < conditionValues.length; i += 2) {
            @SuppressWarnings("unchecked")
            K key = (K) conditionValues[i];
            Command value = (Command) conditionValues[i + 1];

            m_conditionMap.put(key, value);

            addRequirements(value.getRequirements());
        }
    }

    public MultiConditionalCommand(Command defaultCommand, Supplier<K> condition, ConditionCommandEntry<K>[] conditionCommandEntries) {
        m_selectedCommand = defaultCommand;

        m_conditionMap = new HashMap<>();

        for (ConditionCommandEntry<K> entry : conditionCommandEntries) {
            m_conditionMap.put(entry.getKey(), entry.getValue());

            addRequirements(entry.getValue().getRequirements());
        }
    }
    
    public MultiConditionalCommand<K> withCondition(K key, Command command) {
        m_conditionMap.put(key, command);
        addRequirements(command.getRequirements());

        return this;
    }
    
    public MultiConditionalCommand<K> withCondition(ConditionCommandEntry<K> entry) {
        m_conditionMap.put(entry.getKey(), entry.getValue());
        addRequirements(entry.getValue().getRequirements());

        return this;
    }

    @Override
    public void initialize() {
        if(m_conditionMap.containsKey(m_conditionSupplier.get())) {
            m_conditionMap.forEach((key, command) -> {
                if (key.equals(m_conditionSupplier.get())) {
                    m_selectedCommand = command;
                }
            });
        }

        m_selectedCommand.initialize();
    }
    
    @Override
    public void execute() {
        m_selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return m_selectedCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        m_runsWhenDisabled = true;
        m_conditionMap.values().forEach(command -> {
            m_runsWhenDisabled = command.runsWhenDisabled() ? m_runsWhenDisabled : false;
        });
        return m_runsWhenDisabled;
    }
    
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        m_interruptionBehavior = InterruptionBehavior.kCancelIncoming;
        m_conditionMap.values().forEach(command -> {
            m_interruptionBehavior = command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf ? InterruptionBehavior.kCancelSelf : m_interruptionBehavior;
        });
        return m_interruptionBehavior;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        m_conditionMap.forEach((condition, command) -> {
            builder.addStringProperty(condition.toString(), command::getName, null);
        });
        
        builder.addStringProperty(
            "selected",
            () -> {
                if (m_selectedCommand == null) {
                    return "null";
                } else {
                    return m_selectedCommand.getName();
                }
            },
            null);
    }

}

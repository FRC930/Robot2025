package frc.robot.subsystems.algaeendeffector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.util.LoggedTunableNumber;

/**
 * A {@link edu.wpi.first.wpilibj2.command.Subsystem} to control the roller motors for our Algae End Effector.
 * Contains an {@link AlgaeEndEffectorIO} to communicate with either the hardware or the simulation.
 */
public class AlgaeEndEffector extends SubsystemBase {
  public static LoggedTunableNumber ALGAE_DISTANCE_THRESHOLD_HIGH = new LoggedTunableNumber("Sensors/AlgaeEndEffector/SENSORTHRESHOLDHIGH", 4.0);
  public static LoggedTunableNumber ALGAE_DISTANCE_THRESHOLD_LOW = new LoggedTunableNumber("Sensors/AlgaeEndEffector/SENSORTHRESHOLDLOW", 1.7);
  private final AlgaeEndEffectorIO m_IO;

  private final AlgaeEndEffectorInputsAutoLogged m_logged = new AlgaeEndEffectorInputsAutoLogged();
  private final Debouncer m_algaeDebouncer = new Debouncer(0.15, DebounceType.kBoth);

  /**
   * Constructs an {@link AlgaeEndEffector} with the {@link AlgaeEndEffectorIO} specified.
   * @param io A specified IO of either {@link AlgaeEndEffectorIONova}, {@link AlgaeEndEffectorIOSim}, or {@link AlgaeEndEffectorIOTalonFX}. Automatically specified when running Sim or Real.
   */
  public AlgaeEndEffector(AlgaeEndEffectorIO io) {
    m_IO = io;
    m_logged.angularVelocity = DegreesPerSecond.mutable(0);
    m_logged.supplyCurrent = Amps.mutable(0);
    m_logged.torqueCurrent = Amps.mutable(0);
    m_logged.voltage = Volts.mutable(0);
    m_logged.voltageTarget = Volts.mutable(0);
    m_logged.algaeDistance = Inches.mutable(100);
  }

  /**
   * A function to specify the {@link AlgaeEndEffectorIO} to set the voltage to {@code target}.
   * @param target The specific {@link Voltage} to set the {@link AlgaeEndEffectorIO}'s target to. The AlgaeEndEffector motor(s) will attempt to reach the set voltage.
   */
  private void setTarget(Voltage target) {
    m_IO.setTarget(target);
  }

  /**
   * A factory to return a new {@link InstantCommand} to call {@link AlgaeEndEffector#setTarget}.
   * @param volts The {@link DoubleSupplier} returning the specific {@link Voltage} in {@link Volts} to set the motor's target to. The DoubleSupplier is called every time the command is called.
   * @return The constructed {@link InstantCommand}.
   */
  public Command getNewSetVoltsCommand(DoubleSupplier volts) {
    return new InstantCommand(() -> {
      setTarget(Volts.of((volts.getAsDouble())));
    }, this);
  }

  /**
   * A factory to return a new {@link InstantCommand} to call {@link AlgaeEndEffector#setTarget}.
   * @param i The {@link double} specifying the specific {@link Voltage} in {@link Volts} to set the motor's target to. Remember that the double is specified once on command declaration, and not each time the command is run.
   * @return The constructed {@link InstantCommand}.
   */
  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(() -> {
      setTarget(Volts.of(i));
    }, this);
  }

  /**
   * A function to decide if there is currently an Algae held in the Algae End Effector, using distance sensors.
   * @return The {@link boolean} of whether there is currently an Algae in the End Effector.
   */
  public boolean hasAlgae() {
    return m_algaeDebouncer.calculate(
      m_logged.algaeDistance.lte(Inches.of(CoralEndEffector.CORAL_DISTANCE_THRESHOLD_HIGH.get()))
      &&
      m_logged.algaeDistance.gte(Inches.of(CoralEndEffector.CORAL_DISTANCE_THRESHOLD_LOW.get()))
    );
  }

  /**
   * A factory to return a {@link Trigger} that activates when there is an Algae held in the Algae End Effector. It uses the {@link AlgaeEndEffector#hasAlgae} method.
   * @return A constructed Trigger activating if the End Effector has an Algae.
   */
  public Trigger hasAlgaeTrigger() {
    return new Trigger(this::hasAlgae);
  }

  /**
   * {@inheritDoc}
   * {@summary Updates all of the LoggedTunableNumbers and AlgaeEndEffectorInputsAutoLogged values.}
   */
  @Override
  public void periodic() {
    m_IO.updateInputs(m_logged);
    Logger.processInputs("RobotState/AlgaeEndEffector", m_logged);
    if(Constants.tuningMode) {
      Logger.recordOutput("RobotState/AlgaeEndEffector/hasAlgae", hasAlgaeTrigger());
    }
  }
}

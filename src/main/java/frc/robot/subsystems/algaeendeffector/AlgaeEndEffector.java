package frc.robot.subsystems.algaeendeffector;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.util.LoggedTunableNumber;

/**
 * <h1>Algae End Effector</h1>
 * <p>Controls the rollers on the end of our algae end effector</p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class AlgaeEndEffector extends SubsystemBase {
  public static LoggedTunableNumber ALGAE_DISTANCE_THRESHOLD_HIGH = new LoggedTunableNumber("Sensors/AlgaeEndEffector/SENSORTHRESHOLDHIGH", 4.0);
  public static LoggedTunableNumber ALGAE_DISTANCE_THRESHOLD_LOW = new LoggedTunableNumber("Sensors/AlgaeEndEffector/SENSORTHRESHOLDLOW", 1.7);
  private AlgaeEndEffectorIO m_IO;

  AlgaeEndEffectorInputsAutoLogged logged = new AlgaeEndEffectorInputsAutoLogged();
  private Debouncer algaeDebouncer = new Debouncer(0.15, DebounceType.kBoth);

  public AlgaeEndEffector(AlgaeEndEffectorIO io) {
    m_IO = io;
    logged.angularVelocity = DegreesPerSecond.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltage = Volts.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.algaeDistance = Inches.mutable(100);
  }

  private void setTarget(Voltage target) {
    m_IO.setTarget(target);
  }

  public Command getNewSetVoltsCommand(DoubleSupplier volts) {
    return new InstantCommand(() -> {
      setTarget(Volts.of((volts.getAsDouble())));
    }, this);
  }

  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(() -> {
      setTarget(Volts.of(i));
    }, this);
  }

  public boolean hasAlgae() {
    return algaeDebouncer.calculate(
      logged.algaeDistance.lte(Inches.of(CoralEndEffector.CORAL_DISTANCE_THRESHOLD_HIGH.get()))
      &&
      logged.algaeDistance.gte(Inches.of(CoralEndEffector.CORAL_DISTANCE_THRESHOLD_LOW.get()))
    );
  }

  public Trigger hasAlgaeTrigger() {
    return new Trigger(this::hasAlgae);
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/AlgaeEndEffector", logged);
    if(Constants.tuningMode) {
      Logger.recordOutput("RobotState/AlgaeEndEffector/hasAlgae", hasAlgaeTrigger());
    }
  }
}

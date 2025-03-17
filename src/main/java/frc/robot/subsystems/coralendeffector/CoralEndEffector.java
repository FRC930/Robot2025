
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;

/**
 * <h1>Coral end effector</h1>
 * <p>Controls the rollers on the end of our coral end effector</p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class CoralEndEffector extends SubsystemBase {
  public static LoggedTunableNumber CORAL_DISTANCE_THRESHOLD = new LoggedTunableNumber("Coral End Effector/CoralSensorDistanceInches", 0.1);

  private CoralEndEffectorIO m_IO;
  private CoralEndEffectorInputsAutoLogged logged = new CoralEndEffectorInputsAutoLogged();

  public CoralEndEffector(CoralEndEffectorIO io) {
    m_IO = io;
    logged.angularVelocity = DegreesPerSecond.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.voltage = Volts.mutable(0);
    logged.hasCoral = false;
  }

  private void setTarget(Voltage target) {
    m_IO.setTarget(target);
  }

  public Command getNewSetVoltsCommand(LoggedTunableNumber volts) {
    return new InstantCommand(() -> {
      setTarget(Volts.of((volts.get())));
    }, this);
  }
  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(() -> {
      setTarget(Volts.of(i));
    }, this);
  }

  public Trigger hasCoralTrigger() {
    return new Trigger(() -> logged.hasCoral);
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Coral End Effector", logged);
  }
}

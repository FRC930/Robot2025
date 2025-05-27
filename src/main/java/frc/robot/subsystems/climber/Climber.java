
package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;

/**
 * <h1>Coral end effector</h1>
 * <h3>(Climber)</h3>
 * <p>Controls the rollers on the end of our coral end effector</p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class Climber extends SubsystemBase {
  private ClimberIO m_ClimberIO;

  ClimberInputsAutoLogged loggedclimber = new ClimberInputsAutoLogged();

  public Climber(ClimberIO ClimberIO) {
    m_ClimberIO = ClimberIO;
    loggedclimber.angle = Degrees.mutable(0);
    loggedclimber.angularVelocity = DegreesPerSecond.mutable(0);
    loggedclimber.supplyCurrent = Amps.mutable(0);
    loggedclimber.torqueCurrent = Amps.mutable(0);
    loggedclimber.voltageSetPoint = Volts.mutable(0);
    loggedclimber.voltage = Volts.mutable(0);
    loggedclimber.servoTarget = Degrees.mutable(0);
    loggedclimber.servoPos = Degrees.mutable(0);
  }

  public void setTarget(Voltage target) {
    m_ClimberIO.setTarget(target);
  }

  public void setServoTarget(Angle angle) {
    m_ClimberIO.setServoTarget(angle);
  }

  public Command getNewSetVoltsCommand(LoggedTunableNumber volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of((volts.get())));
        },
        this);
  }
  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(i));
        },
        this);
  }
  public Command getNewSetServoAngleCommand(double angle) {
    return new InstantCommand(
        () -> {
          setServoTarget(Degrees.of(angle));
        },
        this);
  }

  public Command getNewStopClimberCommand() {
    return new InstantCommand(()-> {
      m_ClimberIO.stop();
    });
  }

  
  public Trigger getNewGreaterThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(() -> {
      return loggedclimber.angle.in(Degrees) > angle.getAsDouble();
    });
  }

  public Trigger getNewGreaterThanAngleTrigger(Double angle) {
    return getNewGreaterThanAngleTrigger(() -> angle);
  }

  public Trigger getNewLessThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(() -> {
      return loggedclimber.angle.in(Degrees) < angle.getAsDouble();
    });
  }

  public Trigger getNewLessThanAngleTrigger(Double angle) {
    return getNewLessThanAngleTrigger(() -> angle);
  }

  @Override
  public void periodic() {
    m_ClimberIO.updateInputs(loggedclimber);
    Logger.processInputs("RobotState/Climber", loggedclimber);
    Logger.recordOutput("RobotState/Climber/angleDegrees", loggedclimber.angle.in(Degrees));
  }
}


package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    loggedclimber.angularVelocity = DegreesPerSecond.mutable(0);
    loggedclimber.supplyCurrent = Amps.mutable(0);
    loggedclimber.torqueCurrent = Amps.mutable(0);
    loggedclimber.voltageSetPoint = Volts.mutable(0);
    loggedclimber.voltage = Volts.mutable(0);
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

  @Override
  public void periodic() {
    m_ClimberIO.updateInputs(loggedclimber);
    Logger.processInputs("RobotState/Climber", loggedclimber);
  }
}

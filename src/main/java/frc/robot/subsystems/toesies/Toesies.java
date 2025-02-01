package frc.robot.subsystems.toesies;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.toesies.ToesiesInputsAutoLogged;
import frc.robot.util.LoggedTunableNumber;

/**
 * <h1>Algae end effector</h1>
 * <h3>(toesies)</h3>
 * <p>Controls the rollers on the end of our algae end effector</p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class Toesies extends SubsystemBase {
  private ToesiesIO m_ToesiesIO;

  ToesiesInputsAutoLogged loggedtoesies = new ToesiesInputsAutoLogged();

  public Toesies(ToesiesIO toesiesIO) {
    m_ToesiesIO = toesiesIO;
    loggedtoesies.angularVelocity = DegreesPerSecond.mutable(0);
    loggedtoesies.supplyCurrent = Amps.mutable(0);
    loggedtoesies.torqueCurrent = Amps.mutable(0);
    loggedtoesies.voltageSetPoint = Volts.mutable(0);
  }
  public Command getNewSetVoltsCommand(LoggedTunableNumber volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of((volts.get())));
        },
        this);
  }
  public void setTarget(Voltage target) {
    m_ToesiesIO.setTarget(target);
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
    m_ToesiesIO.updateInputs(loggedtoesies);
    Logger.processInputs("RobotState/Toesies", loggedtoesies);
  }
}

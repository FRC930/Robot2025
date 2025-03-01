package frc.robot.subsystems.algaeendeffector;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.algaeendeffector.ToesiesInputsAutoLogged;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

/**
 * <h1>Algae End Effector</h1>
 * <h3>(toesies)</h3>
 * <p>Controls the rollers on the end of our algae end effector</p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class AlgaeEndEffector extends SubsystemBase {
  private AlgaeEndEffectorIO m_IO;

  ToesiesInputsAutoLogged logged = new ToesiesInputsAutoLogged();


  public AlgaeEndEffector(AlgaeEndEffectorIO toesiesIO) {
    m_IO = toesiesIO;
    logged.angularVelocity = DegreesPerSecond.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltage = Volts.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.sensorDistance = Meters.mutable(0);
  }

  /**
   * PLACEHOLDER
   * gets if the end effector is currently holding a coral
   * Please replace with an actual method, and change all usages to reflect
   * TODO: Replace
   * @return
   */
  public BooleanSupplier placeholderGetHasAlgaeSupplier() {
    return () -> false;
  }

  //Creates methods that can be used in commands
  public Command getNewSetVoltsCommand(DoubleSupplier volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of((volts.getAsDouble())));
        },
        this);
  }

  public void setTarget(Voltage target) {
    m_IO.setTarget(target);
  }

  public Distance getDistance() {
    return m_IO.getDistance();
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
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Toesies", logged);
  }
}

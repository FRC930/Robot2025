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
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

/**
 * <h1>Algae End Effector</h1>
 * <p>Controls the rollers on the end of our algae end effector</p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class AlgaeEndEffector extends SubsystemBase {
  public static LoggedTunableNumber ALGAE_DISTANCE_THRESHOLD = new LoggedTunableNumber("Algae End Effector/AlgaeSensorDistanceInches", 0.1);
  private AlgaeEndEffectorIO m_IO;

  AlgaeEndEffectorInputsAutoLogged logged = new AlgaeEndEffectorInputsAutoLogged();

  public AlgaeEndEffector(AlgaeEndEffectorIO io) {
    m_IO = io;
    logged.angularVelocity = DegreesPerSecond.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltage = Volts.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.hasAlgae = false;
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

  public Trigger hasAlgaeTrigger() {
    return new Trigger(() -> logged.hasAlgae);
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Algae End Effector", logged);
  }
}

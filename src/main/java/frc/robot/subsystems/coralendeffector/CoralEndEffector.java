
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;

/**
 * <h1>Coral end effector</h1>
 * <h3>(fingeys)</h3>
 * <p>Controls the rollers on the end of our coral end effector</p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class CoralEndEffector extends SubsystemBase {
  private CoralEndEffectorIO m_IO;

  private static final Distance SENSOR_TRIGGER_DISTANCE = Inches.of(5.0);

  FingeysInputsAutoLogged logged = new FingeysInputsAutoLogged();

  public CoralEndEffector(CoralEndEffectorIO fingeysIO) {
    m_IO = fingeysIO;
    logged.angularVelocity = DegreesPerSecond.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.voltage = Volts.mutable(0);
    logged.sensorDistance = Meters.mutable(0);
  }

  /**
   * PLACEHOLDER
   * gets if the end effector is currently holding a coral
   * Please replace with an actual method, and change all usages to reflect
   * TODO: Replace
   * @return
   */
  public BooleanSupplier placeholderGetHasCoralSupplier() {
    return () -> false;
  }

  public void setTarget(Voltage target) {
    m_IO.setTarget(target);
  }
  public Distance getDistance() {
    return m_IO.getDistance();
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

  /**
   * Generate trigger for waitUntils
   * 
   * @param untilHas set to true to activate trigger when end effector obtains a gamepiece (intake), set to false to activate when end effector loses a gamepiece (score/outake)
   * @return a trigger that activates when the end effector either gets or loses a gamepiece
   */
  public Trigger getHasGamepieceTrigger() {
    return new Trigger(
      () -> getDistance().lte(SENSOR_TRIGGER_DISTANCE)
    );
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/Fingeys", logged);
  }
}

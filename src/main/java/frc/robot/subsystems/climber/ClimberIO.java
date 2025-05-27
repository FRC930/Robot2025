package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {
    public MutAngle angle;
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public MutAngle servoTarget;
    public MutAngle servoPos;
  }

  public void setTarget(Voltage target);

  public void setServoTarget(Angle angle);

  /**
   * Takes a set of inputs, retrieves the current values of these inputs, then updates the given
   * input set.
   *
   * <p>
   */
  public void updateInputs(ClimberInputs input);

  public void stop();
}

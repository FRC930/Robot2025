package frc.robot.subsystems.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public static class WristInputs {
    public MutAngle wristAngle;
    public MutAngularVelocity wristAngularVelocity;
    public MutAngle wristSetPoint;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Angle target);

  public void updateInputs(WristInputs inputs);
}

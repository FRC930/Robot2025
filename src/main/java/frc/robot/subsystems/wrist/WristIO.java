package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;

public interface WristIO {

  @AutoLog
  public static class WristInputs {
    public MutAngle wristAngle;
    public MutAngularVelocity wristAngularVelocity;
    public MutAngle wristSetPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Angle target);

  public void applyCoastMode();

  public void updateInputs(WristInputs inputs);

  public void setGains(Gains gains);
}

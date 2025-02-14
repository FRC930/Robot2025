package frc.robot.subsystems.intakeextender;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtenderIO {

  @AutoLog
  public static class IntakeExtenderInputs {
    public MutAngle Angle;
    public MutAngularVelocity IntakeExtenderAngularVelocity;
    public MutAngle IntakeExtenderSetPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Angle target);

  public void updateInputs(IntakeExtenderInputs inputs);
  
  public void setGains(Gains gains);
}

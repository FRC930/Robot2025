package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.Gains;

public interface ArmJointIO {

  @AutoLog
  public static class ArmInputs {
    public MutAngle angle;
    public MutAngularVelocity angularVelocity;
    public MutAngle setPoint;
    public MutAngle internalSetPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Angle target);

  public void setTargetWithSlot(Angle target, int slot);

  /**
   * Takes a set of inputs, retrieves the current values of these inputs, then updates the given
   * input set.
   */
  public void updateInputs(ArmInputs input);

  public void stop();

  /**
   * Returns the constants associated with this instance of the arm.
   * @return
   */
  public ArmJointConstants getConstants();

  public void setGains(Gains gains,int slot);
}

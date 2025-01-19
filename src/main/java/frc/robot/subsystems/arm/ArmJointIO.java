package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmJointIO {

  @AutoLog
  public static class ArmInputs {
    /** The time in seconds from the FPGA start and the creation of this set of inputs */
    public double timestamp;

    public MutAngle jointAngle;
    public MutAngularVelocity jointAngularVelocity;
    public MutAngle jointSetPoint;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Angle target);

  /**
   * Takes a set of inputs, retrieves the current values of these inputs, then updates the given
   * input set.
   *
   * <p>
   */
  public void updateInputs(ArmInputs input);

  public void periodic();

  public void stop();
}

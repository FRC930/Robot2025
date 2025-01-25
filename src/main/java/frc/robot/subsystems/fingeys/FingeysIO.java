package frc.robot.subsystems.fingeys;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.constants.ArmJointConstants;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface FingeysIO {

  @AutoLog
  public static class FingeysInputs {
    /** The time in seconds from the FPGA start and the creation of this set of inputs */
    public double timestamp;

    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Voltage target);

  /**
   * Takes a set of inputs, retrieves the current values of these inputs, then updates the given
   * input set.
   *
   * <p>
   */
  public void updateInputs(FingeysInputs input);

  public void stop();
}

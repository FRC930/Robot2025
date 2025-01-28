package frc.robot.subsystems.fingeys;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FingeysIO {

  @AutoLog
  public static class FingeysInputs {
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

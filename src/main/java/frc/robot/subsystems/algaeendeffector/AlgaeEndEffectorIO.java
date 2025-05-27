package frc.robot.subsystems.algaeendeffector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface AlgaeEndEffectorIO {

  @AutoLog
  public static class AlgaeEndEffectorInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public MutDistance algaeDistance;
  }

  public void setTarget(Voltage target);

  /**
   * Takes a set of inputs, retrieves the current values of these inputs, then updates the given
   * input set.
   *
   * <p>
   */
  public void updateInputs(AlgaeEndEffectorInputs input);

  public void stop();
}

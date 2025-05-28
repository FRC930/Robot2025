package frc.robot.subsystems.algaeendeffector;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/**
 * A base IO interface for the Algae End Effector.
 * Specifies what functions an Algae IO implementation should be able to do.
 * Includes LoggedTunableNumbers for logging the Algae End Effector's values to AdvantageScope.
 */
public interface AlgaeEndEffectorIO {

  /**
   * A simple structure to hold a set of variables regarding the Algae End Effector's motors.
   */
  @AutoLog
  public static class AlgaeEndEffectorInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltage;
    public MutVoltage voltageTarget;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
    public MutDistance algaeDistance;
  }

  /**
   * A function to set the motor's target, or running voltage, to a specified amount.
   * @param target The specified {@link Voltage} to set the motor's target to.
   */
  public void setTarget(Voltage target);

  /**
   * A function that mutates the passed {@link AlgaeEndEffectorInputs} based on the current state of the Algae End Effector.
   * @param input A set of {@link AlgaeEndEffectorInputs} with values mutably changed according to the current real state of the Algae End Effector.
   */
  public void updateInputs(AlgaeEndEffectorInputs input);

  /**
   * A function to stop the Algae End Effector roller from spinning.
   */
  public void stop();
}

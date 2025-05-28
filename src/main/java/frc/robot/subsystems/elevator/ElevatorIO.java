package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;

/**
 * A base IO interface for the Elevator.
 * Specifies what functions an Elevator IO implementation should be able to do.
 * Includes LoggedTunableNumbers for logging the Elevator's values to AdvantageScope.
 */
public interface ElevatorIO {

  /**
   * A simple structure to hold a set of variables regarding the Elevator's state.
   */
  @AutoLog
  public static class ElevatorInputs {
    public MutDistance distance;
    public MutLinearVelocity velocity;
    public MutDistance setPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  /**
   * A function that mutates the passed {@link ElevatorInputs} based on the current state of the Elevator.
   * @param input A set of {@link ElevatorInputs} with values to be mutably changed according to the current real state of the Elevator.
   */
  public void updateInputs(ElevatorInputs input);

  /**
   * A function to completely stop and brake the movement of the elevator.
   */
  public void stop();

  /**
   * A function to set the target absolute distance for the elevator to be at.
   * @param dist The {@link Distance} above zero (elevator resting at bottom) for the elevator to raise.
   */
  public void setTarget(Distance dist);

  /**
   * A function to set the PIDs (represented by {@link Gains}) of the elevator.
   * @param gains The set of PIDs to set the elevator's motors to.
   */
  public void setGains(Gains gains);
}

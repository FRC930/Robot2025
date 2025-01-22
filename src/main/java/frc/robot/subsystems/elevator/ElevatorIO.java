package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorInputs {
    public double timestamp;

    public MutDistance elevatorDistance;
    public MutVelocity elevatorVelocity;
    public MutDistance elevatorSetPoint;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void setTarget(Distance target, Angle angle);

  public void updateInputs(ElevatorInputs input);

  public void stop();
}

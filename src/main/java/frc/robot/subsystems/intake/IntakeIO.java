package frc.robot.subsystems.intake;


import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public MutAngularVelocity angularVelocity;
    public MutVoltage voltageSetPoint;
    public MutVoltage voltage;
    public MutCurrent supplyCurrent;
    public MutCurrent statorCurrent;
    public MutDistance sensorDistance;
  }

  public void setTarget(Voltage setpoint);
  /**
   * Gets a supplier which will always return the current distance of the intake sensor
   */
  public Supplier<Distance> intakeSensorBro();

  public void updateInputs(IntakeInputs input);

  public void stop();
}

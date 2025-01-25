package frc.robot.subsystems.intake;


import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.fingeys.FingeysIO.FingeysInputs;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    /** The time in seconds from the FPGA start and the creation of this set of inputs */
    public double timestamp;

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

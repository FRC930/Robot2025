package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IntakeIO {
  public void setSpeed(double speed);
  /**
   * Does the sensor lol.
   *
   * <p>A trigger that sense when the sensor senses coral in the inteake
   *
   * @return
   */
  public Trigger intakeSensorBro();
}

package frc.robot.subsystems.intake;

public class Intake {
  private IntakeIO intakeIO;

  public void setSpeed(double speed) {
    intakeIO.setSpeed(speed);
  }
}

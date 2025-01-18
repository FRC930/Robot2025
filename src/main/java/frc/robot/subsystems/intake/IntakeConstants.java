package frc.robot.subsystems.intake;

public class IntakeConstants {
  private IntakeConstants instance;

  private IntakeConstants() {}

  public IntakeConstants getInstance() {
    if (instance == null) {
      instance = new IntakeConstants();
    }

    return instance;
  }
}

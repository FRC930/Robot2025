package frc.robot.subsystems.intakeextender;

public class IntakeExtenderConstants {
    private IntakeExtenderConstants instance;

  private IntakeExtenderConstants() {}

  public IntakeExtenderConstants getInstance() {
    if (instance == null) {
      instance = new IntakeExtenderConstants();
    }

    return instance;
  }
}

package frc.robot.subsystems.arm;

public class ArmConstants {
  private ArmConstants instance;

  private ArmConstants() {}

  public ArmConstants getInstance() {
    if (instance == null) {
      instance = new ArmConstants();
    }

    return instance;
  }
}

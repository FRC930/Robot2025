package frc.robot.subsystems.arm;

public class ArmJointConstants {
  private ArmJointConstants instance;

  private ArmJointConstants() {}

  public ArmJointConstants getInstance() {
    if (instance == null) {
      instance = new ArmJointConstants();
    }

    return instance;
  }
}

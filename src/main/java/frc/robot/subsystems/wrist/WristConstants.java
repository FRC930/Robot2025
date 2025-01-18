package frc.robot.subsystems.wrist;

public class WristConstants {
  private WristConstants instance;

  private WristConstants() {}

  public WristConstants getInstance() {
    if (instance == null) {
      instance = new WristConstants();
    }

    return instance;
  }
}

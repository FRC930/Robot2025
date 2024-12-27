package frc.robot;

import frc.robot.util.VirtualSubsystem;

public class RobotState extends VirtualSubsystem {
  private static RobotState instance;

  private final String key;

  private RobotState(String key) {
    this.key = key;
  }

  public static RobotState instance() {
    if (instance == null) {
      instance = new RobotState("measured");
    }
    return instance;
  }

  @Override
  public void periodic() {
    visualize();
  }

  private void visualize() {
    /*
     * 3D visualization of robot here
     */
  }
}

package frc.robot.subsystems.elevator;

public class ElevatorConstants {
  private ElevatorConstants instance;

  private ElevatorConstants() {}

  public ElevatorConstants getInstance() {
    if (instance == null) {
      instance = new ElevatorConstants();
    }

    return instance;
  }
}

package frc.robot.subsystems.fingeys;

public class FingeysConstants {
  private FingeysConstants instance;

  private FingeysConstants() {}

  public FingeysConstants getInstance() {
    if (instance == null) {
      instance = new FingeysConstants();
    }

    return instance;
  }
}

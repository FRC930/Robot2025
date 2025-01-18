package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorOutput {}

  public static class ElevatorInput {}

  public ElevatorOutput getOutputs();

  public void updateInputs(ElevatorInput input);

  public void periodic();
}

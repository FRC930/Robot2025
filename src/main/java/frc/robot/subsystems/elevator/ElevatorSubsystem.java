package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorIO m_ElevatorIO;

  public ElevatorSubsystem(ElevatorIO elevatorIO) {
    m_ElevatorIO = elevatorIO;
  }
}

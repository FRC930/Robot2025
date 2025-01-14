package frc.robot.subsystems.arm;

public class ArmSubsystem {
  private ArmIO m_ArmIO;

  public ArmSubsystem(ArmIO armIO) {
    m_ArmIO = armIO;
  }

  public void SetAngle1(double angle) {
    m_ArmIO.SetAngle1(angle);
  }

  // @Logged
  // public void GetAngle1 (double angle) {
  //     m_ArmIO.getAngle1();
  // }

  // @Logged
  // public void GetAngle2 (double angle, double Velocity) {
  //     m_ArmIO.getAngle2();
  //     m_ArmIO.getVelocity2();
  // }

  public void SetAngle2(double angle) {
    m_ArmIO.SetAngle2(angle);
  }
}

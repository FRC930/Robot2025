package frc.robot.subsystems.arm;

public interface ArmIO {
  public void SetAngle1(double angle);

  public void SetAngle2(double angle);

  public double getAngle1();

  public double getAngle2();

  public double getVelocity1();

  public double getVelocity2();

  public void periodic();
}

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class ArmSubsystem extends SubsystemBase {
  private ArmIO m_ArmIO;

  public ArmSubsystem(ArmIO armIO) {
    m_ArmIO = armIO;
  }

  public void SetAngle1(double angle) {
    ArmIO.ArmInput input = new ArmIO.ArmInput();
    input.joint1Setpoint = Optional.of(Angle.ofRelativeUnits(angle, Degrees));
    input.joint2Setpoint = Optional.empty();
    m_ArmIO.updateInputs(input);
  }

  // @Logged
  // public void GetAngle1 (double angle) {
  // m_ArmIO.getAngle1();
  // }

  // @Logged
  // public void GetAngle2 (double angle, double Velocity) {
  // m_ArmIO.getAngle2();
  // m_ArmIO.getVelocity2();
  // }

  public void SetAngle2(double angle) {
    ArmIO.ArmInput input = new ArmIO.ArmInput();
    input.joint1Setpoint = Optional.empty();
    input.joint2Setpoint = Optional.of(Angle.ofRelativeUnits(angle, Degrees));
    m_ArmIO.updateInputs(input);
  }

  public Command getNewSetAngle1Command(double angle) {
    return new InstantCommand(
        () -> {
          SetAngle1(angle);
        });
  }

  public Command getNewSetAngle2Command(double angle) {
    return new InstantCommand(
        () -> {
          SetAngle2(angle);
        });
  }

  @Override
  public void periodic() {
    m_ArmIO.periodic();
  }
}

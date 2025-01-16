package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmOutput {
    public Angle joint1Angle;
    public Angle joint2Angle;
    public AngularVelocity joint1AngularVelocity;
    public AngularVelocity joint2AngularVelocity;
    public Angle joint1SetPoint;
    public Angle joint2SetPoint;
  }

  public static class ArmInput {
    public Optional<Angle> joint1Setpoint;
    public Optional<Angle> joint2Setpoint;
  }

  public ArmOutput getOutputs();

  public void updateInputs(ArmInput input);

  // public void SetAngle1(double angle);

  // public void SetAngle2(double angle);

  // public double getAngle1();

  // public double getAngle2();

  // public double getVelocity1();

  // public double getVelocity2();

  public void periodic();
}

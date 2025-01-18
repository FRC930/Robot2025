package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

  @AutoLog
  public static class WristInputs {

    public MutAngle joint3Angle;
    public MutAngularVelocity joint3AngularVelocity;
    public MutAngle joint3SetPoint;
  }

  public void setTarget(Angle target);

  public WristInputs updateInputs(WristInputs inputs);

  public void periodic();
}

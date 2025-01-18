package frc.robot.subsystems.fingeys;

import edu.wpi.first.units.measure.Angle;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface FingeysIO {

  @AutoLog
  public static class FingeysOutput {
    public Angle jointAngle;
    public Angle jointSetPoint;
  }

  public static class FingeysInput {
    public Optional<Angle> jointSetpoint;
  }

  public FingeysOutput getOutputs();

  public void updateInputs(FingeysInput input);

  public void periodic();
}

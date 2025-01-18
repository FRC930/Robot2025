package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  private WristIO m_wristIO;

  public WristSubsystem(WristIO wristIO) {
    m_wristIO = wristIO;
  }

  public Command newWristTurnCommand(double amount) {
    return new InstantCommand(
        () -> {
          m_wristIO.setTarget(Angle.ofRelativeUnits(amount, Degrees));
        },
        this);
  }
}

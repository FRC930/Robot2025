package frc.robot.subsystems.intakeextendor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.subsystems.wrist.WristIO;

public class IntakeExtender extends SubsystemBase {
  private Intakeextendor m_intakeextendorIO;

  intakeextendor loggedwrist = new IntakeextendorAutoLogged();

  //WRIST WEIGHT
  //2.173 lbs
  public intakeextendor(Intakeextendor IntakeextendorIO) {
    m_WristIO = wristIO;
    loggedintakeextendor.intakeextendorAngle = Degrees.mutable(0);
    loggedwrist.intakeextendorAngularVelocity = DegreesPerSecond.mutable(0);
    loggedwrist.intakeSetPoint = Degrees.mutable(0);
    loggedwrist.supplyCurrent = Amps.mutable(0);
    loggedwrist.torqueCurrent = Amps.mutable(0);
    loggedwrist.voltageSetPoint = Volts.mutable(0);

    RobotState.instance().setWristSource(loggedwrist.wristAngle);
  }

  public void setAngle(Angle angle) {
    m_WristIO.setTarget(angle);
  }

  public Command getNewWristTurnCommand(double i) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(i));
        },
        this);
  }

  public Trigger getNewAtAngleTrigger(Angle angle,Angle tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(angle.baseUnitMagnitude(), loggedwrist.wristAngle.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    });
  }

  @Override
  public void periodic() {
    m_WristIO.updateInputs(loggedintake);
    Logger.processInputs("RobotState/Wrist", loggedintake);
  }
}
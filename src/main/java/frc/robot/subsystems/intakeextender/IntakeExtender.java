package frc.robot.subsystems.intakeextendor;

import static edu.wpi.first.units.Units.*;

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
  private IntakeExtender m_intakeextenderIO;

  Intakeextender loggedintakeextender = new IntakeextenderAutoLogged();

  //WRIST WEIGHT
  //2.173 lbs
  public intakeextender(Intakeextender IntakeextenderIO) {
    m_IntakeextenderIO = IntakeextenderIO;
    loggedintakeextendor.intakeextendorAngle = Degrees.mutable(0);
    loggedintakeextender.intakeextendorAngularVelocity = DegreesPerSecond.mutable(0);
    loggedintakeextender.intakeSetPoint = Degrees.mutable(0);
    loggedintakeextender.supplyCurrent = Amps.mutable(0);
    loggedintakeextender.torqueCurrent = Amps.mutable(0);
    loggedintakeextender.voltageSetPoint = Volts.mutable(0);

    RobotState.instance().setWristSource(loggedintakeextender.intakeextenderAngle);
  }

  public void setAngle(Angle angle) {
    m_IntakeExtenderIO.setTarget(angle);
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
      return MathUtil.isNear(angle.baseUnitMagnitude(), loggedintakeextender.intakeextenderAngle.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    });
  }

  @Override
  public void periodic() {
    m_IntakeExtenderIO.updateInputs(loggedintakeextender);
    Logger.processInputs("RobotState/Wrist", loggedintakeextender);
  }
}
package frc.robot.subsystems.intakeextender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.RobotState;

public class IntakeExtender extends SubsystemBase {
  private IntakeExtenderIO m_intakeextenderIO;

  IntakeExtenderInputsAutoLogged loggedintakeExtender = new IntakeExtenderInputsAutoLogged();

  //WRIST WEIGHT
  //2.173 lbs
  public IntakeExtender(IntakeExtenderIO intakeExtenderIO) {
    m_intakeextenderIO = intakeExtenderIO;
    loggedintakeExtender.Angle = Degrees.mutable(0);
    loggedintakeExtender.IntakeExtenderAngularVelocity = DegreesPerSecond.mutable(0);
    loggedintakeExtender.IntakeExtenderSetPoint = Degrees.mutable(0);
    loggedintakeExtender.supplyCurrent = Amps.mutable(0);
    loggedintakeExtender.torqueCurrent = Amps.mutable(0);
    loggedintakeExtender.voltageSetPoint = Volts.mutable(0);

    RobotState.instance().setIntakeExtenderSource(loggedintakeExtender.Angle);
  }

  public void setAngle(Angle angle) {
    m_intakeextenderIO.setTarget(angle);
  }
  
  public Command getNewIntakeExtenderTurnCommand(LoggedTunableNumber angle) {
    return new InstantCommand(
      () -> {
        setAngle(Degrees.of((angle.get())));
      },
      this);
  }

  public Command getNewIntakeExtenderTurnCommand(double i) {
    return new InstantCommand(
      () -> {
        setAngle(Degrees.of(i));
      },
      this);
  }

  public Trigger getNewAtAngleTrigger(Angle angle,Angle tolerance) {
     return new Trigger(() -> {
      return MathUtil.isNear(angle.baseUnitMagnitude(), loggedintakeExtender.Angle.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    });
  }

  @Override
  public void periodic() {
    m_intakeextenderIO.updateInputs(loggedintakeExtender);
    Logger.processInputs("RobotState/IntakeExtender", loggedintakeExtender);
  }
}
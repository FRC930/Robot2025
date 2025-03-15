package frc.robot.subsystems.intakeextender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

public class IntakeExtender extends SubsystemBase {
  private IntakeExtenderIO m_intakeextenderIO;
  double groundIntakeMinDeg = 0;
  double groundIntakeMaxDeg = 90;

  IntakeExtenderInputsAutoLogged loggedintakeExtender = new IntakeExtenderInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableGains = new LoggedTunableGainsBuilder(
    "IntakeExtender", 
    120.0, 0, 0, 
    0, 0, 0, 0, 
    2.0, 1.0, 0, 0, 0
  );
  
  public IntakeExtender(IntakeExtenderIO intakeExtenderIO) {
    m_intakeextenderIO = intakeExtenderIO;
    loggedintakeExtender.Angle = Degrees.mutable(0);
    loggedintakeExtender.IntakeExtenderAngularVelocity = DegreesPerSecond.mutable(0);
    loggedintakeExtender.IntakeExtenderSetPoint = Degrees.mutable(0);
    loggedintakeExtender.supplyCurrent = Amps.mutable(0);
    loggedintakeExtender.torqueCurrent = Amps.mutable(0);
    loggedintakeExtender.voltageSetPoint = Volts.mutable(0);
    loggedintakeExtender.voltage = Volts.mutable(0);

    RobotState.instance().setIntakeExtenderSource(loggedintakeExtender.Angle);
  }

  public Supplier<Angle> getAngleSupplier() {
    return ()->loggedintakeExtender.Angle;
  }

  public void setAngle(Angle angle) {
    m_intakeextenderIO.setTarget(angle);
  }
  
  public Command getNewIntakeExtenderTurnCommand(LoggedTunableNumber angle) {
    return new InstantCommand(
      () -> {
        setAngle(Degrees.of((MathUtil.clamp(angle.get(), groundIntakeMinDeg, groundIntakeMaxDeg))));
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
    tunableGains.ifGainsHaveChanged((gains) -> this.m_intakeextenderIO.setGains(gains));
    m_intakeextenderIO.updateInputs(loggedintakeExtender);
    Logger.processInputs("RobotState/IntakeExtender", loggedintakeExtender);
  }
}
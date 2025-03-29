package frc.robot.subsystems.intakeextender;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.mutable.MutableMeasureBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

public class IntakeExtender extends SubsystemBase {
  private IntakeExtenderIO m_intakeextenderIO;
  double groundIntakeMinDeg = -180;
  double groundIntakeMaxDeg = 0;

  IntakeExtenderInputsAutoLogged loggedintakeExtender = new IntakeExtenderInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableGains = new LoggedTunableGainsBuilder(
    "IntakeExtender", 
    177.0, 0, 0.0, 
    0, 0, 0, 0, 
    1.0, 6.5, 0, 0, 0
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

    // RobotState.instance().setIntakeExtenderSource(loggedintakeExtender.Angle);
  }

  public Supplier<Angle> getAngleSupplier() {
    return ()->loggedintakeExtender.Angle;
  }

  
  private void setAngle(Angle angle) {
    m_intakeextenderIO.setTarget(angle);
  }
  
  public Command getNewIntakeExtenderTurnCommand(DoubleSupplier angle) {
    return new PrintCommand("I would run intake extender if it still existed");
  }

  public Command getNewIntakeExtenderTurnCommand(double i) {
    return new PrintCommand("I would run intake extender if it still existed");
  }

  // public Trigger getNewAtAngleTrigger(Angle angle,Angle tolerance) {
  //   return new Trigger(() -> {
  //     return MathUtil.isNear(angle.baseUnitMagnitude(), loggedintakeExtender.Angle.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
  //   });
  // }

  public Trigger getNewAtAngleTrigger(DoubleSupplier angle, Angle tolerance) {
    return new Trigger(() -> {
      System.out.println("We're checking the intake extender which doesn't exist lmao");
      return true;
    });
  }

  public Trigger getNewAtAngleTrigger(Supplier<Angle> angle, Angle tolerance) {
    return new Trigger(() -> {
      System.out.println("We're checking the intake extender which doesn't exist lmao");
      return true;
    });
  }

  public Trigger getNewLessThanAngleTrigger(Supplier<Angle> angle) {
    return new Trigger(() -> {
      System.out.println("We're checking the intake extender which doesn't exist lmao");
      return true;
    });
  }

  public Trigger getNewLessThanAngleTrigger(DoubleSupplier degrees) {
    return new Trigger(() -> {
      System.out.println("We're checking the intake extender which doesn't exist lmao");
      return true;
    });
  }

  public Trigger getNewGreaterThanAngleTrigger(Supplier<Angle> angle) {
    return new Trigger(() -> {
      System.out.println("We're checking the intake extender which doesn't exist lmao");
      return true;
    });
  }

  public Trigger getNewGreaterThanAngleTrigger(DoubleSupplier degrees) {
    return new Trigger(() -> {
      System.out.println("We're checking the intake extender which doesn't exist lmao");
      return true;
    });
  }


  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_intakeextenderIO.setGains(gains));
    m_intakeextenderIO.updateInputs(loggedintakeExtender);
    Logger.processInputs("RobotState/IntakeExtender", loggedintakeExtender);
  }
}
package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

public class Wrist extends SubsystemBase {
  private WristIO m_WristIO;

  WristInputsAutoLogged loggedwrist = new WristInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableGains = new LoggedTunableGainsBuilder(
    "Wrist", 
    50, 0, 0, 
    0, 0, 0, 0, 
    5.0, 10.0, 0.0, 0, 0
  );

  //WRIST WEIGHT
  //2.173 lbs

  //WRISt POSITIONS
  // 0 - Horizontal
  // 90 - Vertical
  public Wrist(WristIO wristIO) {
    m_WristIO = wristIO;
    loggedwrist.wristAngle = Degrees.mutable(0);
    loggedwrist.wristAngularVelocity = DegreesPerSecond.mutable(0);
    loggedwrist.wristSetPoint = Degrees.mutable(0);
    loggedwrist.supplyCurrent = Amps.mutable(0);
    loggedwrist.torqueCurrent = Amps.mutable(0);
    loggedwrist.voltageSetPoint = Volts.mutable(0);
    loggedwrist.voltage = Volts.mutable(0);

    this.m_WristIO.setGains(tunableGains.build());

    RobotState.instance().setWristSource(loggedwrist.wristAngle);
  }

  public Supplier<Angle> getAngleSupplier() {
    return ()->loggedwrist.wristAngle;
  }

  public void setAngle(Angle angle) {
    m_WristIO.setTarget(angle);
  }

  public Command getNewWristTurnCommand(DoubleSupplier angle) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(angle.getAsDouble()));
        },
        this);
  }

  public Command getNewApplyCoastModeCommand() {
    return new InstantCommand(
        () -> {
          m_WristIO.applyCoastMode();
        },
        this);
  }

  public Command getNewWristTurnCommand(double i) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(i));
        },
        this);
  }

  /**
   * Returns when the wrist is greater than 'angle'
   * @param angle
   * @return
   */
  public Trigger getNewGreaterThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(() -> loggedwrist.wristAngle.in(Degrees) > angle.getAsDouble());
  }

  /**
   * Returns when the wrist is less than 'angle'
   * @param angle
   * @return
   */
  public Trigger getNewLessThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(() -> loggedwrist.wristAngle.in(Degrees) < angle.getAsDouble());
  }


  public Trigger getNewAtAngleTrigger(Angle angle,Angle tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(angle.baseUnitMagnitude(), loggedwrist.wristAngle.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    });
  }

  public Trigger getNewAtSetpointTrigger() {
    return new Trigger(() -> {
      return MathUtil.isNear(loggedwrist.wristSetPoint.baseUnitMagnitude(), loggedwrist.wristAngle.baseUnitMagnitude(), Degrees.of(0.25).baseUnitMagnitude());
    });
  }

  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_WristIO.setGains(gains));
    m_WristIO.updateInputs(loggedwrist);
    Logger.processInputs("RobotState/Wrist", loggedwrist);
  }
}

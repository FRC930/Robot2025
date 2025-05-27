package frc.robot.subsystems.wrist;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;

public class Wrist extends SubsystemBase {
  private WristIO m_WristIO;

  WristInputsAutoLogged loggedwrist = new WristInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableGains = new LoggedTunableGainsBuilder(
    "Gains/Wrist/", 
    50, 0, 0, 
    0, 0, 0, 0, 
    5.0, 10.0, 0.0, 0, 0
  );

  /** Setup the initial values for the {@link Wrist}. */
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

  /**
   * Creates a {@link Supplier} that will dynamically return the CANCoder's reported wrist position, in degrees.
   * @return A {@link Supplier} returning the CANCoder's reported wrist position on supplier call.
   */
  public Supplier<Angle> getAngleSupplier() {
    return ()->loggedwrist.wristAngle;
  }
  
  /**
   * Sets this instance's {@link WristIO} target angle to {@code angle}.
   * @param angle The angle to set the target to.
   */
  public void setAngle(Angle angle) {
    m_WristIO.setTarget(angle);
  }

  /**
   * A command factory that creats an {@link InstantCommand} to set the wrist's target position to {@code angle}.
   * @param angle A DoubleSupplier for the angle to set the wrist to in degrees.
   * @return A {@link InstantCommand} that sets the wrist's angle based on the given {@link DoubleSupplier}.
   */
  public Command getNewWristTurnCommand(DoubleSupplier angle) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(angle.getAsDouble()));
        },
        this);
  }

  /**
   * Constructs a command that will apply coast mode to the wrist's motor.
   * @return An {@link InstantCommand} to set the wrist motor's braking to coast mode.
   */
  public Command getNewApplyCoastModeCommand() {
    return new InstantCommand(
        () -> {
          m_WristIO.applyCoastMode();
        },
        this);
  }
  
  /**
   * Constructs a command to set the wrist's turn command.
   * @param i The angle, in degrees, to set the wrist's turn target to.
   * @return An {@link InstantCommand} to set the wrist's target position to the passed in parameter.
   */
  public Command getNewWristTurnCommand(double i) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(i));
        },
        this);
  }

  /**
   * Constructs a trigger that dynamically updates to trigger when the wrist's CANCoder angle is greater than the given one.
   * @param angle The angle to check for in the condition.
   * @return The constructed {@link Trigger}.
   */
  public Trigger getNewGreaterThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(() -> loggedwrist.wristAngle.in(Degrees) > angle.getAsDouble());
  }

  /**
   * Constructs a trigger that dynamically updates to trigger when the wrist's CANCoder angle is lesser than the given one.
   * @param angle The angle to check for in the condition.
   * @return The constructed {@link Trigger}.
   */
  public Trigger getNewLessThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(() -> loggedwrist.wristAngle.in(Degrees) < angle.getAsDouble());
  }

  /**
   * Constructs a trigger for when the Wrist CANCoder's reported angle is within a certain threshold.
   * @param angle The angle to check if the CANCoder is within threshold of.
   * @param tolerance The threshold, or error, that the conditional will allow for the trigger.
   * @return The constructed {@link Trigger} with a dynamic {@code MathUtil.isNear(...)} conditional.
   */
  public Trigger getNewAtAngleTrigger(Angle angle, Angle tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(angle.baseUnitMagnitude(), loggedwrist.wristAngle.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    });
  }

  /**
   * Constructs a trigger for when the Wrist CANCoder's reported angle is within a certain threshold.
   * @param angle The angle, in degrees, to check if the CANCoder is within threshold of.
   * @param tolerance The threshold, or error, that the conditional will allow for the trigger.
   * @return The constructed {@link Trigger} with a dynamic {@code MathUtil.isNear(...)} conditional.
   */
  public Trigger getNewAtAngleTrigger(DoubleSupplier angle, Angle tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(angle.getAsDouble(), loggedwrist.wristAngle.in(Degrees), tolerance.in(Degrees));
    });
  }

/**
 * Constructs a trigger for when the Wrist CANCoder's reported angle is at the setpoint within a threshold of 0.25°.
 * @return A {@link Trigger} that triggers whenever the Wrist is at the setpoint, with a threshold.
 */
  public Trigger getNewAtSetpointTrigger() {
    return new Trigger(() -> {
      return MathUtil.isNear(loggedwrist.wristSetPoint.baseUnitMagnitude(), loggedwrist.wristAngle.baseUnitMagnitude(), Degrees.of(0.25).baseUnitMagnitude());
    });
  }

  /**
   * Updates all of the {@link WristIO}'s inputs, and adds them to the {@link Logger} ({@link org.littletonrobotics.junction.Logger}).
   */
  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_WristIO.setGains(gains));
    m_WristIO.updateInputs(loggedwrist);
    Logger.processInputs("RobotState/Wrist", loggedwrist);
  }
}

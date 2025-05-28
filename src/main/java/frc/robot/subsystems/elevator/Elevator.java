package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;

public class Elevator extends SubsystemBase {
  private ElevatorIO m_ElevatorIO;

  ElevatorInputsAutoLogged loggedelevator = new ElevatorInputsAutoLogged();

  public static final double SPOOL_RADIUS = 1.751 / 2.0;

  public static final double INCHES_PER_ROT = (2.0 * Math.PI * SPOOL_RADIUS);
  
  public static final double REDUCTION = (4.0/1.0);

  public LoggedTunableGainsBuilder tunableGains = new LoggedTunableGainsBuilder(
    "Gains/Elevator/", 
    45.0, 0, 10.0, 
    5.0, 30.0, 0.0, 0.0, 
    125.0, 80.0, 0.0, 0.0, 0.0
  );
  
  public Elevator(ElevatorIO elevatorIO) {
    m_ElevatorIO = elevatorIO;
    loggedelevator.distance = Inches.mutable(0);
    loggedelevator.velocity = InchesPerSecond.mutable(0);
    loggedelevator.setPoint = Meters.mutable(0);
    loggedelevator.supplyCurrent = Amps.mutable(0);
    loggedelevator.torqueCurrent = Amps.mutable(0);
    loggedelevator.voltageSetPoint = Volts.mutable(0);
    loggedelevator.voltage = Volts.mutable(0);

    this.m_ElevatorIO.setGains(tunableGains.build());
    RobotState.instance().setElevatorSource(loggedelevator.distance);
  }

  public Supplier<Distance> getDistanceExtendedSupplier() {
    return () -> loggedelevator.distance;
  }

  public void setDistance(Distance target) {
    m_ElevatorIO.setM_target(target);
  }

  public Command getNewSetDistanceCommand(DoubleSupplier distance) {
    return new InstantCommand(
        () -> {
          setDistance(Inches.of(distance.getAsDouble()));
        },
        this);
  }
  /**
   * @param i Inches
   * */
  public Command getNewSetDistanceCommand(double i) {
    return new InstantCommand(
        () -> {
          setDistance(Inches.of(i));
        },
        this);
  }

  /**
   * Constructs a trigger that dynamically updates to trigger when the elevator's CANCoder angle is lesser than the given one.
   * @param angle The angle to check for in the condition.
   * @return The constructed {@link Trigger}.
   */
  public Trigger getNewAtDistanceTrigger(Distance dist, Distance tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(dist.baseUnitMagnitude(), loggedelevator.distance.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    });
  }

  /**
   * Constructs a trigger for when the elevator's CANCoder's reported angle is within a certain threshold.
   * @param angle The angle, in degrees, to check if the CANCoder is within threshold of.
   * @param tolerance The threshold, or error, that the conditional will allow for the trigger.
   * @return The constructed {@link Trigger} with a dynamic {@code MathUtil.isNear(...)} conditional.
   */
  public Trigger getNewAtDistanceTrigger(DoubleSupplier dist, DoubleSupplier tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(dist.getAsDouble(), loggedelevator.distance.in(Inches), tolerance.getAsDouble());
    });
  }

  /**
   * Constructs a trigger that dynamically updates to trigger when the elevator's CANCoder angle is greater than the given one.
   * @param angle The angle to check for in the condition.
   * @return * the constructed {@link trigger}
   */
  public Trigger getNewGreaterThanDistanceTrigger(DoubleSupplier distance) {
    return new Trigger(() -> {
      return loggedelevator.distance.in(Inches) > distance.getAsDouble();
    });
  }

  /**
   * Constructs a trigger that dynamically updates to trigger when the elevator's CANCoder angle is lesser than the given one.
   * @param angle The angle to check for in the condition.
   * @return The constructed {@link Trigger}.
   */
  public Trigger getNewLessThanDistanceTrigger(DoubleSupplier distance) {
    return new Trigger(() -> {
      return loggedelevator.distance.in(Inches) < distance.getAsDouble();
    });
  }

  /**
   * Updates all of the {@link elevatorIO}'s inputs, and adds them to the {@link Logger} ({@link org.littletonrobotics.junction.Logger}).
   */
  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_ElevatorIO.setGains(gains));
    m_ElevatorIO.updateInputs(loggedelevator);
    Logger.processInputs("RobotState/Elevator", loggedelevator);
  }
}

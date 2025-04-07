package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

public class ArmJoint extends SubsystemBase {

  private ArmJointIO m_armJointIO;

  private final ArmJointConstants m_constants;

  ArmInputsAutoLogged m_loggedArm = new ArmInputsAutoLogged();

  public LoggedTunableGainsBuilder tunableSlot0Gains;
  public LoggedTunableGainsBuilder tunableSlot1Gains;
  
  private Supplier<Angle> previousJointAngleSupplier;

  public ArmJoint(ArmJointIO armJointIO, Optional<ArmJoint> previousJoint) {
    m_armJointIO = armJointIO;
    m_loggedArm.angle = Degrees.mutable(0);
    m_loggedArm.angularVelocity = DegreesPerSecond.mutable(0);
    m_loggedArm.setPoint = Degrees.mutable(0);
    m_loggedArm.supplyCurrent = Amps.mutable(0);
    m_loggedArm.torqueCurrent = Amps.mutable(0);
    m_loggedArm.voltageSetPoint = Volts.mutable(0);
    m_loggedArm.voltage = Volts.mutable(0);
    m_loggedArm.internalSetPoint = Degrees.mutable(0);
    
    previousJointAngleSupplier = () -> Degrees.zero();
    previousJoint.ifPresent((j)->{previousJointAngleSupplier = j.getAngleSupplier();});

    // The weirdest hackiest part of this constants setup.
    // Since we don't have an instance of this class in the IO, but we do have an instance of the IO in this class, we have 
    //    to pass the constants instance back up from the IO to get an instance of it in the subsystem.
    m_constants = armJointIO.getConstants();
    m_constants.mechanismSimCallback.accept(m_loggedArm.angle);
    tunableSlot0Gains = m_constants.TalonFXGainsSlot0;
    tunableSlot1Gains = m_constants.TalonFXGainsSlot1;
    m_loggedArm.angle.mut_replace(m_constants.StartingAngle);
  }

  public Supplier<Angle> getAngleSupplier() {
    return ()->m_loggedArm.angle;
  }

  public void setAngle(Angle angle) {
    m_armJointIO.setTarget(angle);
  }

  public void setAngleWithSlot(Angle angle, int slot) {
    m_armJointIO.setTargetWithSlot(angle, slot);
  }
  
  public Command getNewSetAngleCommand(DoubleSupplier degrees) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of((degrees.getAsDouble())));
        },
        this);
  }
  /**Degrees*/
  public Command getNewSetAngleCommand(double i) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(i));
        },
        this);
  }

  public Command getNewSetAngleWithSlotCommand(DoubleSupplier degrees, int slot) {
    return new InstantCommand(
        () -> {
          setAngleWithSlot(Degrees.of((degrees.getAsDouble())), slot);
        },
        this);
  }
  /**Degrees*/
  public Command getNewSetAngleWithSlotCommand(double i, int slot) {
    return new InstantCommand(
        () -> {
          setAngleWithSlot(Degrees.of(i), slot);
        },
        this);
  }

  public Trigger getNewAtAngleTrigger(Angle angle,Angle tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(angle.baseUnitMagnitude(), m_loggedArm.angle.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    });
  }

  public Trigger getNewAtAngleTrigger(Supplier<Angle> angle,Supplier<Angle> tolerance) {
    return getNewAtAngleTrigger(()->angle.get().in(Degrees), ()->tolerance.get().in(Degrees));
  }

  public Trigger getNewAtAngleTrigger(DoubleSupplier angleDegrees, DoubleSupplier toleranceDegrees) {
    return new Trigger(() -> {
      return MathUtil.isNear(angleDegrees.getAsDouble(), m_loggedArm.angle.in(Degrees), toleranceDegrees.getAsDouble());
    });
  }

  /**
   * @deprecated lmao don't use it. GREG HAS SPOKEN!
   * @return
   */
  @Deprecated
  public Trigger getNewAtSetpointTrigger() {
    return new Trigger(() -> {
      return MathUtil.isNear(m_loggedArm.setPoint.baseUnitMagnitude(), m_loggedArm.angle.baseUnitMagnitude(), Degrees.of(0.25).baseUnitMagnitude());
    });
  }

  /**
   * Returns when this joint is greater than 'angle' away from the forward horizontal
   * @param angle
   * @return
   */
  public Trigger getNewGreaterThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(() -> {
      return m_loggedArm.angle.in(Degrees) > angle.getAsDouble();
    });
  }

  /**
   * Returns when this joint is greater than 'angle' away from the forward horizontal
   * @param angle
   * @return
   */
  public Trigger getNewGreaterThanAngleTrigger(Double angle) {
    return getNewGreaterThanAngleTrigger(() -> angle);
  }

  /**
   * Returns when this joint is less than 'angle' away from the forward horizontal
   * @param angle
   * @return
   */
  public Trigger getNewLessThanAngleTrigger(DoubleSupplier angle) {
    return new Trigger(() -> {
      return m_loggedArm.angle.in(Degrees) < angle.getAsDouble();
    });
  }

  /**
   * Returns when this joint is less than 'angle' away from the forward horizontal
   * @param angle
   * @return
   */
  public Trigger getNewLessThanAngleTrigger(Double angle) {
    return getNewLessThanAngleTrigger(() -> angle);
  }

  @Override
  public void periodic() {
    tunableSlot0Gains.ifGainsHaveChanged((gains) -> this.m_armJointIO.setGains(gains,0));
    tunableSlot1Gains.ifGainsHaveChanged((gains) -> this.m_armJointIO.setGains(gains,1));
    m_armJointIO.updateInputs(m_loggedArm);
    Logger.processInputs("RobotState/" + m_constants.LoggedName, m_loggedArm);
  }
}
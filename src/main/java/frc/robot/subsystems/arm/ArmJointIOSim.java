package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.Gains;

public class ArmJointIOSim implements ArmJointIO {

  //Sim properties
  private ArmFeedforward ff;
  private final ProfiledPIDController controller;
  private final SingleJointedArmSim sim;
  private final ArmJointConstants m_Constants;
  
  //Control Signal
  private Voltage m_appliedVoltage = Volts.mutable(0.0);

  public ArmJointIOSim(ArmJointConstants constants) {
    sim = new SingleJointedArmSim(
                      DCMotor.getKrakenX60Foc(1),
                      constants.MotorToSensorGearing * constants.SensorToMechanismGearing,
                      SingleJointedArmSim.estimateMOI(constants.Length.in(Meters), constants.Weight.in(Kilograms)),
                      constants.Length.in(Meters),
                      -9999,
                      9999,
                      false, //TODO: Tune FFs to allow this to be true
                      constants.StartingAngle.in(Radians),
                      0.001,
                      0.001);
    controller = new ProfiledPIDController(constants.SimGains.kP, constants.SimGains.kI, constants.SimGains.kD, new Constraints(constants.MaxVelocity.in(DegreesPerSecond), constants.MaxAcceleration.in(DegreesPerSecondPerSecond)));
    ff = new ArmFeedforward(constants.SimGains.kS, constants.SimGains.kG, constants.SimGains.kV, constants.SimGains.kA);
    m_Constants = constants;
    controller.setGoal(constants.StartingAngle.in(Degrees));
  }

  @Override
  public void setTarget(Angle target) {
    controller.setGoal(target.in(Degrees));
  }

  /**
   * Updates the applied voltage to drive the arm towards the noted position
   */
  private void updateVoltageSetpoint() {
    Angle currentAngle = Radians.of(sim.getAngleRads());

    Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees)));
    Voltage feedForwardVoltage =
        Volts.of(
            ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    runVolts(effort);
  }

  /**
   * Sets the applied voltage to the volts
   * @param volts
   */
  private void runVolts(Voltage volts) {
    this.m_appliedVoltage = volts;
  }

  @Override
  public void updateInputs(ArmInputs input) {
    // updinputs
    input.angle.mut_replace(Degrees.convertFrom(sim.getAngleRads(), Radians), Degrees);
    input.angularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.setPoint.mut_replace(controller.getGoal().position, Degrees);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(m_appliedVoltage);
    input.voltage.mut_replace(m_appliedVoltage);

    // Periodic
    updateVoltageSetpoint();
    sim.setInputVoltage(m_appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  @Override
  public void stop() {
    Angle currentAngle = Radians.of(sim.getAngleRads());
    controller.reset(currentAngle.in(Degrees));
    runVolts(Volts.of(0));
  }

  @Override
  public ArmJointConstants getConstants() {
    return m_Constants;
  }

  public void setGains(Gains gains) {
    DriverStation.reportWarning("Sim gains tuning not implemented", true);
  }
}

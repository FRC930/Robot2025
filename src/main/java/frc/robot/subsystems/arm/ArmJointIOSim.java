package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.subsystems.arm.constants.ArmJointConstants;

public class ArmJointIOSim implements ArmJointIO {

  private Voltage appliedVoltage = Volts.mutable(0.0);

  private ArmFeedforward ff;

  private final ProfiledPIDController controller;

  private final SingleJointedArmSim sim;

  private final ArmJointConstants m_Constants;

  public ArmJointIOSim(int motorId, ArmJointConstants constants) {
    sim = new SingleJointedArmSim(
                      DCMotor.getKrakenX60Foc(1),
                      constants.Gearing,
                      SingleJointedArmSim.estimateMOI(constants.Length.in(Meters), constants.Weight.in(Kilograms)),
                      constants.Length.in(Meters),
                      constants.MinimumAngle.in(Degrees),
                      constants.MaximumAngle.in(Degrees),
                      true,
                      constants.StartingAngle.in(Degrees),
                      0.001,
                      0.001);
    controller = new ProfiledPIDController(constants.SimGains.kP, constants.SimGains.kI, constants.SimGains.kD, new Constraints(constants.MaxVelocity.in(DegreesPerSecond), constants.MaxAcceleration.in(DegreesPerSecondPerSecond)));
    ff = new ArmFeedforward(constants.SimGains.kS, constants.SimGains.kG, constants.SimGains.kV, constants.SimGains.kA);
    m_Constants = constants;
  }

  @Override
  public void setTarget(Angle target) {
    controller.setGoal(new State(target.in(Degrees), 0));
  }

  private void updateVoltageSetpoint() {
    Angle currentAngle = Radians.of(sim.getAngleRads());

    Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees)));
    Voltage feedForwardVoltage =
        Volts.of(
            ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void updateInputs(ArmInputs input) {
    // updinputs
    input.jointAngle.mut_replace(Degrees.convertFrom(sim.getAngleRads(), Radians), Degrees);
    input.jointAngularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.jointSetPoint.mut_replace(controller.getGoal().position, Degrees);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);

    // Periodic
    updateVoltageSetpoint();
    sim.setInputVoltage(appliedVoltage.in(Volts));
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
}

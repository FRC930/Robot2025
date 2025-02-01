package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;

public class ArmJointIOSim implements ArmJointIO {

  private Voltage appliedVoltage = Volts.mutable(0.0);

  private ArmFeedforward ff = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(100000, 100000));

  private final SingleJointedArmSim sim;

  public ArmJointIOSim(int motorId, SingleJointedArmSim armSim) {
    sim = armSim;
  }

  @Override
  public void setTarget(Angle target) {
    Angle currentAngle = Radians.of(sim.getAngleRads());

    Angle setpointAngle = Degrees.of(controller.getSetpoint().position);
    AngularVelocity setpointVelocity = DegreesPerSecond.of(controller.getSetpoint().velocity);

    Voltage controllerVoltage =
        Volts.of(controller.calculate(currentAngle.in(Degrees), setpointAngle.in(Degrees)));
    Voltage feedForwardVoltage =
        Volts.of(ff.calculate(setpointAngle.in(Radians), setpointVelocity.in(RadiansPerSecond)));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void updateInputs(ArmInputs input) {
    input.jointAngle.mut_replace(sim.getAngleRads(), Radians);
    input.jointAngularVelocity.mut_replace(sim.getVelocityRadPerSec(), RadiansPerSecond);
    input.jointSetPoint.mut_replace(controller.getSetpoint().position, Degrees);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);
    periodic();
  }

  @Override
  public void periodic() {
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  @Override
  public void stop() {
    Angle currentAngle = Radians.of(sim.getAngleRads());
    controller.reset(currentAngle.in(Degrees));
    runVolts(Volts.of(0));
  }
}

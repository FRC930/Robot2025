package frc.robot.subsystems.fingeys;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.PhoenixUtil;

public class FingeysIOSim implements FingeysIO {

  private Voltage appliedVoltage = Volts.mutable(0.0);

  private ArmFeedforward ff;

  private final ProfiledPIDController controller;

  private final FlywheelSim sim;

  private final ArmJointConstants m_Constants;

  public FingeysIOSim(int motorId, FingeysConstants constants) {
    sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.05, 1), DCMotor.getKrakenX60Foc(1), 1);
    controller = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0,0.0));
  }

  @Override
  public void setTarget(Angle target) {
    controller.setGoal(new State(target.in(Degrees), 0));
  }

  private void updateVoltageSetpoint() {
    AngularVelocity currentVelocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());

    Voltage controllerVoltage = Volts.of(controller.calculate(currentVelocity.in(DegreesPerSecond)));

    Voltage effort = controllerVoltage;
    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void updateInputs(FingeysInputs input) {
    input.jointAngularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);

    // Periodic
    updateVoltageSetpoint();
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  @Override
  public void setTarget(AngularVelocity target) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setTarget'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }
}

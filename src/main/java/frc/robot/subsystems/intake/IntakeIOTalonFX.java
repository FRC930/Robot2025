package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.subsystems.fingeys.FingeysIO.FingeysInputs;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  public VoltageOut Request;
  public TalonFX Motor;

  public ArmInputs inputs;

  private CANrange rangeSensor;

  public IntakeIOTalonFX(int id,int CANrangeID) {
    Motor = new TalonFX(id);
    Request = new VoltageOut(0.0);
    rangeSensor = new CANrange(CANrangeID);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;
    cfg.CurrentLimits.StatorCurrentLimit = 120;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.voltageSetPoint.mut_replace(
        Voltage.ofRelativeUnits(
            ((VoltageOut) Motor.getAppliedControl()).Output, Volts));
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
    inputs.sensorDistance.mut_replace(rangeSensor.getDistance().getValue());
  }

  @Override
  public void setTarget(Voltage target) {
    Request = Request.withOutput(target);
    Motor.setControl(Request);
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }

  @Override
  public Supplier<Distance> intakeSensorBro() {
    return () -> {
      return rangeSensor.getDistance().getValue();
    }; 
  }
}

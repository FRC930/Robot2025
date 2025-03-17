package frc.robot.subsystems.coralendeffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class CoralEndEffectorIOTalonFX implements CoralEndEffectorIO {
  private VoltageOut request;
  private TalonFX motor;
  private CANrange m_sensor;

  private Voltage m_setPoint = Volts.of(0);

  public CoralEndEffectorIOTalonFX(CanDef canbus, CanDef sensorCanDef) {
    motor = new TalonFX(canbus.id(),canbus.bus());
    request = new VoltageOut(0.0);
    m_sensor = new CANrange(sensorCanDef.id(),sensorCanDef.bus());
    
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 10.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.Voltage.PeakForwardVoltage = 16.0;
    cfg.Voltage.PeakReverseVoltage = 16.0;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(CoralEndEffectorInputs inputs) {
    inputs.angularVelocity.mut_replace(motor.getVelocity().getValue());
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValue());
    inputs.torqueCurrent.mut_replace(motor.getStatorCurrent().getValue());
    inputs.hasCoral = m_sensor.getDistance().getValue().lt(Inches.of(CoralEndEffector.CORAL_DISTANCE_THRESHOLD.get()));
  }

  @Override
  public void setTarget(Voltage target) {
    request = request.withOutput(target);
    motor.setControl(request);
    m_setPoint = target;
  }

  @Override
  public void stop() {
    motor.setControl(new StaticBrake());
  }
}

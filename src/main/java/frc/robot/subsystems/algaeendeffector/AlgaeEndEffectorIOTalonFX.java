package frc.robot.subsystems.algaeendeffector;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class AlgaeEndEffectorIOTalonFX implements AlgaeEndEffectorIO {
  private VoltageOut request;
  private TalonFX motor;
  private CANrange m_sensor;

  private LinearFilter m_filter = LinearFilter.movingAverage(10);
  private Voltage m_setPoint = Volts.of(0);

  public AlgaeEndEffectorIOTalonFX(CanDef motorCanDef, CanDef sensorCanDef) {
    motor = new TalonFX(motorCanDef.id(),motorCanDef.bus());
    request = new VoltageOut(0.0);
    m_sensor = new CANrange(sensorCanDef.id(),sensorCanDef.bus());

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.StatorCurrentLimit = 100.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(cfg));

    CANrangeConfiguration cr_cfg = new CANrangeConfiguration();
    cr_cfg.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    PhoenixUtil.tryUntilOk(5, ()->m_sensor.getConfigurator().apply(cr_cfg));
  }

  @Override
  public void updateInputs(AlgaeEndEffectorInputs inputs) {
    inputs.angularVelocity.mut_replace(motor.getVelocity().getValue());
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValue());
    inputs.torqueCurrent.mut_replace(motor.getStatorCurrent().getValue());
    inputs.algaeDistance.mut_replace(m_sensor.getDistance().getValue());
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

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

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

/**
 * An implementation of the {@link AlgaeEndEffectorIO} for real life TalonFX motors.
 * @see {@link AlgaeEndEffectorIO}
 * {@inheritDoc}
 */
public class AlgaeEndEffectorIOTalonFX implements AlgaeEndEffectorIO {
  private VoltageOut m_request;
  private final TalonFX m_motor;
  private final CANrange m_sensor;

  private Voltage m_target = Volts.of(0);

  /**
   * A constructor to create a {@link AlgaeEndEffectorIOTalonFX} with specified CAN definitions.
   * @param motorCanDef The CAN definition for the TalonFX motors.
   * @param sensorCanDef The CAN definition for the CANrange sensors.
   */
  public AlgaeEndEffectorIOTalonFX(CanDef motorCanDef, CanDef sensorCanDef) {
    m_motor = new TalonFX(motorCanDef.id(),motorCanDef.bus());
    m_request = new VoltageOut(0.0);
    m_sensor = new CANrange(sensorCanDef.id(),sensorCanDef.bus());

    configureTalons();
  }

  /**
   * A helper to reconfigure the TalonFX motors.
   */
  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.StatorCurrentLimit = 100.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> m_motor.getConfigurator().apply(cfg));

    CANrangeConfiguration cr_cfg = new CANrangeConfiguration();
    cr_cfg.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    PhoenixUtil.tryUntilOk(5, ()->m_sensor.getConfigurator().apply(cr_cfg));
  }

  /**
   * @see {@link AlgaeEndEffectorIO#updateInputs}
   * {@inheritDoc}
   */
  @Override
  public void updateInputs(AlgaeEndEffectorInputs inputs) {
    inputs.angularVelocity.mut_replace(m_motor.getVelocity().getValue());
    inputs.voltageTarget.mut_replace(m_target);
    inputs.voltage.mut_replace(m_motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(m_motor.getSupplyCurrent().getValue());
    inputs.torqueCurrent.mut_replace(m_motor.getStatorCurrent().getValue());
    inputs.algaeDistance.mut_replace(m_sensor.getDistance().getValue());
  }

  /**
   * @see {@link AlgaeEndEffectorIO#setTarget}
   * {@inheritDoc}
   */
  @Override
  public void setTarget(Voltage target) {
    m_request = m_request.withOutput(target);
    m_motor.setControl(m_request);
    m_target = target;
  }

  /**
   * @see {@link AlgaeEndEffectorIO#stop}
   * {@inheritDoc}
   */
  @Override
  public void stop() {
    m_motor.setControl(new StaticBrake());
  }
}

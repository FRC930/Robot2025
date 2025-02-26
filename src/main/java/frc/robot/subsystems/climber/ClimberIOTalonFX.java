package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class ClimberIOTalonFX implements ClimberIO {
  private VoltageOut Request;
  private TalonFX Motor;
  private Servo m_servo;

  private Voltage m_setPoint = Voltage.ofBaseUnits(0, Volts);

  public ClimberIOTalonFX(CanDef canbus) {
    Motor = new TalonFX(canbus.id(),canbus.bus());
    Request = new VoltageOut(0.0);
    m_servo = new Servo(0);
    
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.StatorCurrentLimit = 80.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.Voltage.PeakForwardVoltage = 16.0;
    cfg.Voltage.PeakReverseVoltage = 16.0;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
  }

  @Override
  public void setTarget(Voltage target) {
    Request = Request.withOutput(target);
    Motor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void setServoTarget(Angle angle) {
    m_servo.setAngle(angle.in(Degrees));
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}

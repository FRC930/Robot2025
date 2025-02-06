package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class WristIOTalonFX implements WristIO {
  public PositionVoltage Request;
  public TalonFX Motor;
  public CANcoder canCoder;

  public WristIOTalonFX(CanDef canbus,CanDef canCoderDef) {
    Motor= new TalonFX(canbus.id(), canbus.bus());
    Request = new PositionVoltage(0);
    canCoder = new CANcoder(canCoderDef.id(), canCoderDef.bus());

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = 7;
    cfg.Voltage.PeakReverseVoltage = 7;
    cfg.CurrentLimits.StatorCurrentLimit = 80;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    //Motion magic gains TODO: Add to gains object (probably extend to a TalonFx specific version)
    cfg.MotionMagic.MotionMagicCruiseVelocity = 0.25;
    cfg.MotionMagic.MotionMagicAcceleration = 0.5;

    cfg.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.Feedback.RotorToSensorRatio = 9.0;

    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));

    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.MagnetOffset = -0.223 - 0.012;//UNIT: ROTATIONS
    //AdvantageScope publishes in radians

    PhoenixUtil.tryUntilOk(5, () -> canCoder.getConfigurator().apply(cc_cfg));
  }

  public void setTarget(Angle target) {
    Request = Request.withPosition(target).withSlot(0);
    Motor.setControl(Request);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    inputs.wristAngle.mut_replace(Motor.getPosition().getValue());
    inputs.wristAngularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.wristSetPoint.mut_replace(
        Angle.ofRelativeUnits(
            PhoenixUtil.getPositionFromController(Motor, 0.0), Rotations));
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }

  @Override
  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(motionMagicConfigs));
  }
}

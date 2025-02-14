package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.Gains;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.PhoenixUtil;

public class ArmJointIOTalonFX implements ArmJointIO {
  public MotionMagicVoltage Request;
  public TalonFX Motor;

  public ArmInputs inputs;
  
  private CANcoder cancoder = null;

  private final ArmJointConstants m_Constants;

  private Angle m_setPoint = Angle.ofRelativeUnits(0, Rotations);

  public ArmJointIOTalonFX(ArmJointConstants constants, InvertedValue motorInversion) {
    m_Constants = constants;
    if(constants.CanCoderProfile != null) {
      this.cancoder = new CANcoder(constants.CanCoderProfile.id(),constants.CanCoderProfile.bus());
    }
    Motor = new TalonFX(constants.LeaderProfile.id(),constants.LeaderProfile.bus());
    Request = new MotionMagicVoltage(constants.StartingAngle);
    configureTalons(motorInversion);
  }

  private void configureTalons(InvertedValue motorInversion) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.CurrentLimits.SupplyCurrentLimit = m_Constants.SupplyCurrentLimit.in(Amp);
    cfg.CurrentLimits.StatorCurrentLimit = m_Constants.TorqueCurrentLimit.in(Amp);
    cfg.MotorOutput.Inverted = motorInversion;
    cfg.Feedback.SensorToMechanismRatio = m_Constants.SensorToMechanismGearing;
    cfg.Feedback.RotorToSensorRatio = m_Constants.MotorToSensorGearing;

    if(cancoder != null) {
      cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
      cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

      CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
      cc_cfg.MagnetSensor.MagnetOffset = m_Constants.CanCoderOffset.in(Rotations);//UNIT: ROTATIONS
      // cc_cfg.MagnetSensor.SensorDirection = SenserDirectionValue.CounterClockwise_Positive; // TODO
      PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cc_cfg));
    }
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
    setGains(Gains.getEmpty());
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.angle.mut_replace(Motor.getPosition().getValue());
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.setPoint.mut_replace(m_setPoint);
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
  }

  @Override
  public void setTarget(Angle target) {
    Request = Request.withPosition(target);
    Motor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }

  @Override
  public ArmJointConstants getConstants() {
    return m_Constants;
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
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
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

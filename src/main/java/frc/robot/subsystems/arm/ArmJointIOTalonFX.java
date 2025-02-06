package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ArmJointIOTalonFX implements ArmJointIO {
  public PositionVoltage Request;
  public TalonFX Motor;

  public ArmInputs inputs;

  private final ArmJointConstants m_Constants;

  public ArmJointIOTalonFX(ArmJointConstants constants, InvertedValue motorInversion) {
    m_Constants = constants;
    Motor = new TalonFX(constants.LeaderProfile.id(), constants.LeaderProfile.bus());
    Request = new PositionVoltage(constants.StartingAngle);
    configureTalons(motorInversion);
  }

  private void configureTalons(InvertedValue motorInversion) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.kP = m_Constants.TalonFXGains.kP;
    cfg.Slot0.kI = m_Constants.TalonFXGains.kI;
    cfg.Slot0.kD = m_Constants.TalonFXGains.kD;
    cfg.Slot0.kG = m_Constants.TalonFXGains.kG;
    cfg.Slot0.kS = m_Constants.TalonFXGains.kS;
    cfg.Slot0.kV = m_Constants.TalonFXGains.kV;
    cfg.Slot0.kA = m_Constants.TalonFXGains.kA;
    cfg.CurrentLimits.SupplyCurrentLimit = m_Constants.SupplyCurrentLimit.in(Amp);
    cfg.CurrentLimits.StatorCurrentLimit = m_Constants.TorqueCurrentLimit.in(Amp);
    cfg.MotorOutput.Inverted = motorInversion;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.angle.mut_replace(Motor.getPosition().getValue());
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    // inputs.setPoint.mut_replace(
    //     Angle.ofRelativeUnits(
    //         ((PositionVoltage) Motor.getAppliedControl()).Position, Rotations));
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }

  @Override
  public void setTarget(Angle target) {
    Request = Request.withPosition(target);
    Motor.setControl(Request);
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

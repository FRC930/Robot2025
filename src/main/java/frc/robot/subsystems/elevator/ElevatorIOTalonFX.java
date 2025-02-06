package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  public ElevatorIOTalonFX() {}


  public static final double SPOOL_RADIUS = 1.0;//TODO: Set this

  public PositionVoltage Request;
  public TalonFX leaderMotor;
  public TalonFX followerMotor;

  public ArmInputs inputs;

  public ElevatorIOTalonFX(CanDef leftDef, CanDef rightDef) {
    leaderMotor = new TalonFX(leftDef.id(), leftDef.bus());
    followerMotor = new TalonFX(rightDef.id(), rightDef.bus());
    Request = new PositionVoltage(0);

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
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfg.Feedback.SensorToMechanismRatio = 3.0;

    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(cfg));

    TalonFXConfiguration cfg2 = new TalonFXConfiguration();
    cfg2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg2.Voltage.PeakForwardVoltage = 7;
    cfg2.Voltage.PeakReverseVoltage = 7;
    cfg2.CurrentLimits.StatorCurrentLimit = 80;
    cfg2.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg2.CurrentLimits.SupplyCurrentLimit = 40;
    cfg2.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg2.Feedback.SensorToMechanismRatio = 3.0;

    PhoenixUtil.tryUntilOk(5, ()->followerMotor.getConfigurator().apply(cfg2));

    //NOTE: SET TO OPPOSE MAIN DIRECTION HERE. DO NOT SET OTHERWISE.
    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(),true));
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.distance.mut_replace(Meters.of(leaderMotor.getPosition().getValue().in(Degrees) * 2 * Math.PI * SPOOL_RADIUS ));
    inputs.velocity.mut_replace(MetersPerSecond.of(leaderMotor.getVelocity().getValue().in(DegreesPerSecond) * 2 * Math.PI * SPOOL_RADIUS));
    // inputs.setPoint.mut_replace(
    //     Distance.ofRelativeUnits(
    //         ((PositionVoltage) leaderMotor.getAppliedControl()).Position, Meters));
    inputs.supplyCurrent.mut_replace(leaderMotor.getStatorCurrent().getValue());
  }

  @Override
  public void setTarget(Distance meters) {
    Request = Request.withPosition(meters.in(Meters)/(2 * Math.PI * SPOOL_RADIUS));
    leaderMotor.setControl(Request);
  }

  @Override
  public void stop() {
    leaderMotor.setControl(new StaticBrake());
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
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(motionMagicConfigs));
  }
}

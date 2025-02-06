package frc.robot.subsystems.intakeextender;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class IntakeExtenderIOTalonFX implements IntakeExtenderIO {
  public PositionVoltage Request;
  public TalonFX Motor;

  public IntakeExtenderIOTalonFX(CanDef canbus) {
    Motor= new TalonFX(canbus.id(), canbus.bus());
    Request = new PositionVoltage(0);

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = 7;
    cfg.CurrentLimits.StatorCurrentLimit = 80;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.Voltage.PeakReverseVoltage = 7;

    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  public void setTarget(Angle target) {
    Request = Request.withPosition(target);
    Motor.setControl(Request);
  }

  @Override
  public void updateInputs(IntakeExtenderInputs inputs) {
    inputs.Angle.mut_replace(Motor.getPosition().getValue());
    inputs.IntakeExtenderAngularVelocity.mut_replace(Motor.getVelocity().getValue());
    // inputs.IntakeExtenderSetPoint.mut_replace(
    //     Angle.ofRelativeUnits(
    //         ((PositionVoltage) Motor.getAppliedControl()).Position, Rotations));
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

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

public class WristIOTalonFX implements WristIO {
  public MotionMagicVoltage request3;
  public LoggedTunableNumber accelerationConfigNumber = new LoggedTunableNumber("Arm/Joint1/kA");

  public TalonFX Motor3;

  public WristIOTalonFX(int Motor3Id) {
    Motor3 = new TalonFX(Motor3Id);
    request3 = new MotionMagicVoltage(0);
    configureTalons();
  }

  private void configureTalons() {
    MotionMagicConfigs mm_cfg = new MotionMagicConfigs();
    mm_cfg.MotionMagicAcceleration = 0.0;
    mm_cfg.MotionMagicCruiseVelocity = 0.0;
    mm_cfg.MotionMagicExpo_kA = 0.0;
    mm_cfg.MotionMagicExpo_kV = 0.0;
    mm_cfg.MotionMagicJerk = 0.0;
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = 7;
    cfg.Voltage.PeakReverseVoltage = 7;
    // cfg.CurrentLimits.;
    // cfg.CurrentLimits.; TODO: Set Current Limits
    PhoenixUtil.tryUntilOk(5, () -> Motor3.getConfigurator().apply(cfg));
    Motor3.getConfigurator().apply(mm_cfg);
  }

  public void setTarget(Angle target) {
    request3 = request3.withPosition(target);
    Motor3.setControl(request3);
  }

  @Override
  public WristInputs updateInputs(WristInputs inputs) {
    inputs.joint3Angle.mut_replace(Motor3.getPosition().getValue());
    inputs.joint3AngularVelocity.mut_replace(Motor3.getVelocity().getValue());
    inputs.joint3SetPoint.mut_replace(
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) Motor3.getAppliedControl()).Position, Rotations));
    return inputs;
  }

  @Override
  public void periodic() {}
}

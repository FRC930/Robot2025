package frc.robot.subsystems.fingeys;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

public class FingeysIOTalonFX implements FingeysIO {

  public MotionMagicVoltage Request;
  public LoggedTunableNumber accelerationConfigNumber = new LoggedTunableNumber("Arm/Joint1/kA");

  public TalonFX Motor;

  public FingeysIOTalonFX(int MotorId) {
    Motor = new TalonFX(MotorId);
    Request = new MotionMagicVoltage(0);
    configureTalons();
  }
  /** configures the real motor: acceleration, velocity, and other stuff */
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
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
    Motor.getConfigurator().apply(mm_cfg);
  }

  @Override
  public FingeysOutput getOutputs() {
    FingeysIO.FingeysOutput output = new FingeysIO.FingeysOutput();
    output.jointAngle = Motor.getPosition().getValue();
    output.jointSetPoint =
        Angle.ofRelativeUnits(((MotionMagicVoltage) Motor.getAppliedControl()).Position, Rotations);
    return output;
  }

  @Override
  public void updateInputs(FingeysInput input) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }
}

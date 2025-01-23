package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.PhoenixUtil;

public class ArmJointIOTalonFX implements ArmJointIO {
  public PositionVoltage Request;
  public TalonFX Motor;

  public ArmInputs inputs;

  private final ArmJointConstants m_Constants;

  public ArmJointIOTalonFX(ArmJointConstants constants) {
    m_Constants = constants;
    Motor = new TalonFX(constants.LeaderProfile.id());
    Request = new PositionVoltage(constants.StartingAngle);
    configureTalons();
  }

  private void configureTalons() {
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
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.jointAngle.mut_replace(Motor.getPosition().getValue());
    inputs.jointAngularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.jointSetPoint.mut_replace(
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) Motor.getAppliedControl()).Position, Rotations));
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
}

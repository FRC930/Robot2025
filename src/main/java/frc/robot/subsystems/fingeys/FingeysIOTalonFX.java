package frc.robot.subsystems.fingeys;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.PhoenixUtil;

public class FingeysIOTalonFX implements FingeysIO {
  public VelocityVoltage Request;
  public TalonFX Motor;

  public ArmInputs inputs;

  private final FingeysConstants m_Constants;

  public FingeysIOTalonFX(FingeysConstants constants) {
    m_Constants = constants;
    Motor = new TalonFX(constants.LeaderProfile.id());
    Request = new VelocityVoltage(constants.StartingAngularVelocity);
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
  public void updateInputs(FingeysInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.velocitySetPoint.mut_replace(
        AngularVelocity.ofRelativeUnits(
            ((VelocityVoltage) Motor.getAppliedControl()).Velocity, RotationsPerSecond));
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }

  @Override
  public void setTarget(AngularVelocity target) {
    Request = Request.withVelocity(target);
    Motor.setControl(Request);
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }

  @Override
  public FingeysConstants getConstants() {
    return m_Constants;
  }
}

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class WristIOTalonFX implements WristIO {
  public PositionVoltage Request;
  public TalonFX Motor;

  public WristIOTalonFX(CanDef canbus) {
    Motor= new TalonFX(canbus.id(), canbus.bus());
    Request = new PositionVoltage(0);

    Motor.setControl(Request);
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

    cfg.Slot0.kP = 1.0;

    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  public void setTarget(Angle target) {
    Request = Request.withPosition(target);
    Motor.setControl(Request);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    inputs.wristAngle.mut_replace(Motor.getPosition().getValue());
    inputs.wristAngularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.wristSetPoint.mut_replace(
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) Motor.getAppliedControl()).Position, Rotations));
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }
}

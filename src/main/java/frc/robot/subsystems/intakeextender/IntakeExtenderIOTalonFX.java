package frc.robot.subsystems.intakeextender;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.PhoenixUtil;

public class IntakeExtenderIOTalonFX implements IntakeExtenderIO {
  public PositionVoltage Request;
  public TalonFX Motor;

  public IntakeExtenderIOTalonFX(int MotorId) {
    Motor= new TalonFX(MotorId);
    Request = new PositionVoltage(0);

    Motor.setControl(Request);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = 7;
    cfg.Voltage.PeakReverseVoltage = 7;

    cfg.Slot0.kP = 1.0;

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
    inputs.IntakeExtenderSetPoint.mut_replace(
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) Motor.getAppliedControl()).Position, Rotations));
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }
}

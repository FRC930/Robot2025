package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil;

public class ArmIOTalonFX implements ArmIO {
  public MotionMagicVoltage Request1;
  public MotionMagicVoltage Request2;
  public LoggedTunableNumber accelerationConfigNumber = new LoggedTunableNumber("Arm/Joint1/kA");

  public TalonFX Motor1;

  public TalonFX Motor2;

  public ArmIOTalonFX(int Motor1Id, int Motor2Id) {
    Motor1 = new TalonFX(Motor1Id);
    Motor2 = new TalonFX(Motor2Id);
    Request1 = new MotionMagicVoltage(0);
    Request2 = new MotionMagicVoltage(0);
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
    PhoenixUtil.tryUntilOk(5, () -> Motor1.getConfigurator().apply(cfg));
    PhoenixUtil.tryUntilOk(5, () -> Motor2.getConfigurator().apply(cfg));
    Motor1.getConfigurator().apply(mm_cfg);
  }

  public void SetAngle1(double angle) {

    Request1 = Request1.withPosition(angle);
    Motor1.setControl(Request1);
  }

  public void SetAngle2(double angle) {
    Request2 = Request2.withPosition(angle);
    Motor2.setControl(Request2);
  }

  public double getAngle1() {
    return Motor1.getPosition().getValueAsDouble();
  }

  public double getAngle2() {
    return Motor2.getPosition().getValueAsDouble();
  }

  public double getVelocity1() {
    return Motor1.getVelocity().getValueAsDouble();
  }

  public double getVelocity2() {
    return Motor1.getVelocity().getValueAsDouble();
  }

  public void periodic() {}

  @Override
  public ArmOutput getOutputs() {
    ArmIO.ArmOutput output = new ArmIO.ArmOutput();
    output.joint1Angle = Motor1.getPosition().getValue();
    output.joint2Angle = Motor2.getPosition().getValue();
    output.joint1AngularVelocity = Motor1.getVelocity().getValue();
    output.joint2AngularVelocity = Motor2.getVelocity().getValue();
    output.joint1SetPoint =
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) Motor1.getAppliedControl()).Position, Rotations);
    output.joint2SetPoint =
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) Motor2.getAppliedControl()).Position, Rotations);
    return output;
  }

  @Override
  public void updateInputs(ArmIO.ArmInput input) {
    if (input.joint1Setpoint.isPresent()) {
      Request1 = Request1.withPosition(input.joint1Setpoint.get());
      Motor1.setControl(Request1);
    }
    if (input.joint2Setpoint.isPresent()) {
      Request2 = Request2.withPosition(input.joint2Setpoint.get());
      Motor2.setControl(Request2);
    }
  }
}

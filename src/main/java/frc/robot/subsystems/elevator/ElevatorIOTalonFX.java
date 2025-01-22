package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  public ElevatorIOTalonFX() {}

  public MotionMagicVoltage Request;
  public TalonFX Motor;

  public ArmInputs inputs;

  public ElevatorIOTalonFX(int Motor1Id) {
    Motor = new TalonFX(Motor1Id);
    Request = new MotionMagicVoltage(0);
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
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(mm_cfg));
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    //Keegan - Turn this into Distance
    inputs.elevatorDistance.mut_replace(Motor.getDistance().getValue());
    inputs.elevatorAngularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.elevatorSetPoint.mut_replace(
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) Motor.getAppliedControl()).Position, Rotations));
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }

  @Override
  public void setTarget(Distance target, Angle angle) {
    Request = Request.withPosition(target);
    Motor.setControl(Request);
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}

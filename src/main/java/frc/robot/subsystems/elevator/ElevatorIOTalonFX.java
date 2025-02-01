package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  public ElevatorIOTalonFX() {}

  public static final double ROTATIONS_TO_DISTANCE = 1.0;//TODO: Set this

  public PositionVoltage Request;
  public TalonFX Motor;

  public ArmInputs inputs;

  public ElevatorIOTalonFX(int Motor1Id) {
    Motor = new TalonFX(Motor1Id);
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

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.elevatorDistance.mut_replace(Meters.of(Motor.getPosition().getValue().in(Degrees) * ROTATIONS_TO_DISTANCE));
    inputs.elevatorVelocity.mut_replace(MetersPerSecond.of(Motor.getVelocity().getValue().in(DegreesPerSecond) *ROTATIONS_TO_DISTANCE));
    inputs.elevatorSetPoint.mut_replace(
        Distance.ofRelativeUnits(
            ((MotionMagicVoltage) Motor.getAppliedControl()).Position, Meters));
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }

  @Override
  public void setTarget(Distance meters) {
    Request = Request.withPosition(meters.in(Meters)/ROTATIONS_TO_DISTANCE);
    Motor.setControl(Request);
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}

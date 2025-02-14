package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

  public ElevatorIOTalonFX() {}

  public MotionMagicVoltage Request;
  public TalonFX leaderMotor;
  public TalonFX followerMotor;

  public ArmInputs inputs;

  private Distance m_setPoint = Distance.ofBaseUnits(0, Inches);

  public ElevatorIOTalonFX(CanDef leftDef, CanDef rightDef) {
    leaderMotor = new TalonFX(leftDef.id(), leftDef.bus());
    followerMotor = new TalonFX(rightDef.id(), rightDef.bus());
    Request = new MotionMagicVoltage(0);

    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = 12;
    cfg.Voltage.PeakReverseVoltage = 12;
    cfg.CurrentLimits.StatorCurrentLimit = 80;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfg.Feedback.SensorToMechanismRatio = Elevator.REDUCTION;

    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(cfg));

    TalonFXConfiguration cfg2 = new TalonFXConfiguration();
    cfg2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg2.Voltage.PeakForwardVoltage = 12;
    cfg2.Voltage.PeakReverseVoltage = 12;
    cfg2.CurrentLimits.StatorCurrentLimit = 80;
    cfg2.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg2.CurrentLimits.SupplyCurrentLimit = 40;
    cfg2.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg2.Feedback.SensorToMechanismRatio = Elevator.REDUCTION;

    PhoenixUtil.tryUntilOk(5, ()->followerMotor.getConfigurator().apply(cfg2));

    //NOTE: SET TO OPPOSE MAIN DIRECTION HERE. DO NOT SET OTHERWISE.
    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(),true));
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    double rotations=leaderMotor.getPosition().getValue().in(Rotations);
    // SmartDashboard.putNumber("ElevatorrotorPos", leaderMotor.getRotorPosition().getValue().in(Rotations));
    // SmartDashboard.putNumber("ElevatorPos", inches);
    inputs.distance.mut_replace(Inches.of(rotations * Elevator.INCHES_PER_ROT));
    // inputs.distance.mut_replace(Inches.of(leaderMotor.getRotorPosition().getValue().in(Rotations)*Elevator.INCHES_PER_ROT));
    inputs.velocity.mut_replace(InchesPerSecond.of(leaderMotor.getVelocity().getValue().in(RotationsPerSecond)));
    inputs.setPoint.mut_replace(m_setPoint);
    inputs.supplyCurrent.mut_replace(leaderMotor.getSupplyCurrent().getValue());
    inputs.voltage.mut_replace(leaderMotor.getMotorVoltage().getValue());
  }

  @Override
  public void setTarget(Distance target) {
    Request = Request.withPosition(target.in(Inches)/Elevator.INCHES_PER_ROT).withSlot(0);
    leaderMotor.setControl(Request);
    m_setPoint = target;
    
  }

  @Override
  public void stop() {
    leaderMotor.setControl(new StaticBrake());
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
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(motionMagicConfigs));
  }
}

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.PhoenixUtil;

/** DO NOT USE, UNFINISHED AND WILL CAUSE AN ERROR */
public class ArmIOSim implements ArmIO {
  public MotionMagicVoltage Request1;
  public MotionMagicVoltage Request2;

  public final double J1_KP = 1.0;
  public final double J1_KD = 1.0;
  public final double J1_GEARING = 1.0;

  public final double J2_KP = 1.0;
  public final double J2_KD = 1.0;
  public final double J2_GEARING = 1.0;

  private final DCMotorSim joint1Sim;
  private final DCMotorSim joint2Sim;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private TalonFX talon1;
  private TalonFX talon2;

  private static final double flwheel_kA = 0.00032;
  private static final double flwheel_kV = 1.0;

  private final LinearSystem<N2, N1, N2> m_flywheelPlant =
      LinearSystemId.createDCMotorSystem(flwheel_kV, flwheel_kA);

  public ArmIOSim(int motor1Id, int motor2Id) {
    joint1Sim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.1, 0.1);
    joint2Sim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.1, 0.1);
    talon1 = new TalonFX(motor1Id, "rio");
    talon2 = new TalonFX(motor2Id, "rio");
    Request1 = new MotionMagicVoltage(0);
    Request2 = new MotionMagicVoltage(0);
    configureTalons();
  }

  private void configureTalons() {
    MotionMagicConfigs mm_cfg = new MotionMagicConfigs();
    mm_cfg.MotionMagicAcceleration = 1.0;
    mm_cfg.MotionMagicCruiseVelocity = 1.0;
    mm_cfg.MotionMagicExpo_kA = 1.0;
    mm_cfg.MotionMagicExpo_kV = 1.0;
    mm_cfg.MotionMagicJerk = 1.0;
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = 7;
    cfg.Voltage.PeakReverseVoltage = 7;
    // cfg.CurrentLimits.;
    // cfg.CurrentLimits.; TODO: Set Current Limits
    PhoenixUtil.tryUntilOk(5, () -> talon1.getConfigurator().apply(cfg));
    PhoenixUtil.tryUntilOk(5, () -> talon2.getConfigurator().apply(cfg));

    PhoenixUtil.tryUntilOk(5, () -> talon1.getConfigurator().apply(mm_cfg));
    PhoenixUtil.tryUntilOk(5, () -> talon2.getConfigurator().apply(mm_cfg));
  }

  // public void SetAngle1(double angle) {
  //   Request1 = Request1.withPosition(angle);
  //   talon1.setControl(Request1);
  // }

  // public void SetAngle2(double angle) {
  //   Request2 = Request2.withPosition(angle);
  //   talon2.setControl(Request2);
  // }

  // public double getAngle1() {
  //   return talon1.getPosition().getValueAsDouble();
  // }

  // public double getAngle2() {
  //   return talon2.getPosition().getValueAsDouble();
  // }

  @Override
  public ArmOutput getOutputs() {
    ArmIO.ArmOutput output = new ArmIO.ArmOutput();
    output.joint1Angle = talon1.getPosition().getValue();
    output.joint2Angle = talon2.getPosition().getValue();
    output.joint1AngularVelocity = talon1.getVelocity().getValue();
    output.joint2AngularVelocity = talon2.getVelocity().getValue();
    output.joint1SetPoint =
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) talon1.getAppliedControl()).Position, Rotations);
    output.joint2SetPoint =
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) talon2.getAppliedControl()).Position, Rotations);
    return output;
  }

  public void periodic() {
    talon1.getSimState().setSupplyVoltage(12.0);
    talon2.getSimState().setSupplyVoltage(12.0);

    // talon1.getSimState().getMotorVoltage()
    // talon2.getSimState().getMotorVoltage()
    joint1Sim.setInputVoltage(joint1Sim.getAngularPosition().times(J1_GEARING).in(Degrees));
    joint2Sim.setInputVoltage(joint2Sim.getAngularPosition().times(J2_GEARING).in(Degrees));

    // 0.02 is default delta secs for commands
    joint1Sim.update(0.02);
    joint2Sim.update(0.02);

    // talon1.getSimState().setRawRotorPosition(joint1Sim.getAngularPosition().times(J1_GEARING));
    // talon2.getSimState().setRawRotorPosition(joint2Sim.getAngularPosition().times(J2_GEARING));
  }

  @Override
  public void updateInputs(ArmIO.ArmInput input) {
    if (input.joint1Setpoint.isPresent()) {
      Request1 = Request1.withPosition(input.joint1Setpoint.get());
      talon1.setControl(Request1);
    }
    if (input.joint2Setpoint.isPresent()) {
      Request2 = Request2.withPosition(input.joint2Setpoint.get());
      talon2.setControl(Request2);
    }
  }
}

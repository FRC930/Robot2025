package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.PhoenixUtil;

public class WristIOSim implements WristIO {
  public MotionMagicVoltage Request3;

  public final double J3_KP = 1.0;
  public final double J3_KD = 1.0;
  public final double J3_GEARING = 1.0;

  private final DCMotorSim joint3Sim;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private TalonFX talon3;

  private static final double flwheel_kA = 0.00032;
  private static final double flwheel_kV = 1.0;

  private final LinearSystem<N2, N1, N2> m_flywheelPlant = LinearSystemId.createDCMotorSystem(flwheel_kV, flwheel_kA);

  public WristIOSim(int motor3Id) {
    joint3Sim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.1, 0.1);
    talon3 = new TalonFX(motor3Id, "rio");
    Request3 = new MotionMagicVoltage(0);
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

    PhoenixUtil.tryUntilOk(5, () -> talon3.getConfigurator().apply(cfg));
    PhoenixUtil.tryUntilOk(5, () -> talon3.getConfigurator().apply(mm_cfg));
  }

  @Override
  public WristInputs updateInputs(WristInputs inputs) {
    inputs.joint3Angle.mut_replace(talon3.getPosition().getValue());
    inputs.joint3AngularVelocity.mut_replace(talon3.getVelocity().getValue());
    inputs.joint3SetPoint.mut_replace(
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) talon3.getAppliedControl()).Position, Rotations));
    return inputs;
  }

  @Override
  public void periodic() {
    talon3.getSimState().setSupplyVoltage(12.0);

    joint3Sim.setInputVoltage(joint3Sim.getAngularPosition().times(J3_GEARING).in(Degrees));

    joint3Sim.update(0.02);
  }

  @Override
  public void setTarget(Angle target) {
    Request3.withPosition(target);
    talon3.setControl(Request3);
  }
}

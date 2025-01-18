package frc.robot.subsystems.fingeys;

import static edu.wpi.first.units.Units.Degrees;

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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.PhoenixUtil;

public class FingeysIOSim implements FingeysIO {

  public MotionMagicVoltage Request;

  public final double J_KP = 1.0;
  public final double J_KD = 1.0;
  public final double J_GEARING = 1.0;

  private final DCMotorSim jointSim;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private TalonFX talon;

  private static final double flwheel_kA = 0.00032;
  private static final double flwheel_kV = 1.0;

  private final LinearSystem<N2, N1, N2> m_flywheelPlant =
      LinearSystemId.createDCMotorSystem(flwheel_kV, flwheel_kA);

  public FingeysIOSim(int motor1Id, int motor2Id) {
    jointSim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.1, 0.1);
    talon = new TalonFX(motor1Id, "rio");
    Request = new MotionMagicVoltage(0);
    configureTalons();
  }
  /** configures the sim motor: acceleration, velocity, and other stuff */
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
    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(cfg));

    PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(mm_cfg));
  }

  @Override
  public FingeysOutput getOutputs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getOutputs'");
  }

  @Override
  public void updateInputs(FingeysInput input) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void periodic() {
    talon.getSimState().setSupplyVoltage(12.0);

    jointSim.setInputVoltage(jointSim.getAngularPosition().times(J_GEARING).in(Degrees));

    // 0.02 is default delta secs for commands
    jointSim.update(0.02);
  }
}

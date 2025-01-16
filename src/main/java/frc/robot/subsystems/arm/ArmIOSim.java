package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** DO NOT USE, UNFINISHED AND WILL CAUSE AN ERROR */
public class ArmIOSim implements ArmIO {
  public MotionMagicVoltage Request1;
  public MotionMagicVoltage Request2;

  public final double J1_KP = 1.0;
  public final double J1_KD = 1.0;

  public final double J2_KP = 1.0;
    public final double J2_KD = 1.0;

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
    joint1Sim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.001, 0.001);
    joint2Sim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.001, 0.001);
    talon1 = new TalonFX(motor1Id);
    talon2 = new TalonFX(motor2Id);
    Request1 = new MotionMagicVoltage(0);
    Request2 = new MotionMagicVoltage(0);
  }

  public void SetAngle1(double angle) {
    Request1 = Request1.withPosition(angle);
    talon1.setControl(Request1);
  }

  public void SetAngle2(double angle) {
    Request2 = Request2.withPosition(angle);
    talon2.setControl(Request2);
  }

  public double getAngle1() {
    return talon1.getPosition().getValueAsDouble();
  }

  public double getAngle2() {
    return talon2.getPosition().getValueAsDouble();
  }

  @Override
  public ArmOutput getOutputs() {
  ArmIO.ArmOutput output = new ArmIO.ArmOutput();
  output.joint1Angle = talon1.getPosition().getValue();
  output.joint2Angle = talon2.getPosition().getValue();
  output.joint1AngularVelocity = talon1.getVelocity().getValue();
  output.joint2AngularVelocity = talon2.getVelocity().getValue();
  output.joint1SetPoint = Angle.ofRelativeUnits(((MotionMagicVoltage)talon1.getAppliedControl()).Position,Rotations);
  output.joint2SetPoint = Angle.ofRelativeUnits(((MotionMagicVoltage)talon2.getAppliedControl()).Position,Rotations); 
  return output;
  }

  public void periodic() {
    talon1.getSimState().setSupplyVoltage(12.0);
    talon2.getSimState().setSupplyVoltage(12.0);

    joint1Sim.setInputVoltage(talon1.getSimState().getMotorVoltage());
    joint2Sim.setInputVoltage(talon2.getSimState().getMotorVoltage());

    // 0.02 is default delta secs for commands
    joint1Sim.update(0.02);
    joint2Sim.update(0.02);
  }

  @Override
  public void updateInputs(Angle Angle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("lmao");
  }
}

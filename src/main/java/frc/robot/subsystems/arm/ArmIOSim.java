package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** DO NOT USE, UNFINISHED AND WILL CAUSE AN ERROR */
public class ArmIOSim implements ArmIO {
  public final double J1_KP = 0.0;
  public final double J1_KD = 0.0;

  public final double J2_KP = 0.0;
  public final double J2_KD = 0.0;

  private final DCMotorSim joint1Sim;
  private final DCMotorSim joint2Sim;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private PIDController joint1Controller = new PIDController(J1_KP, 0, J1_KD);
  private PIDController joint2Controller = new PIDController(J2_KP, 0, J2_KD);

  private static final double flwheel_kA = 0.00032;
  private static final double flwheel_kV = 1.0;

  private final LinearSystem<N2, N1, N2> m_flywheelPlant =
      LinearSystemId.createDCMotorSystem(flwheel_kV, flwheel_kA);

  public ArmIOSim(int motor1Id, int motor2Id) {
    joint1Sim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.001);
    joint2Sim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.001);
    throw new Error("Sim kinda crazy ngl, Harry you figure this one out lol");
  }

  public void SetAngle1(double angle) {
    // TODO: Sim kinda crazy ngl ngl, Harry you figure this one out lol
    throw new Error("Sim kinda craz ngl, Harry you figure this one out lol");
  }

  public void SetAngle2(double angle) {
    // TODO: Sim kinda crazy ngl ngl, Harry you figure this one out lol
    throw new Error("Sim kinda crazy ngl ngl, Harry you figure this one out lol");
  }

  public double getAngle1() {
    // TODO: Sim kinda crazy ngl ngl, Harry you figure this one out lol
    throw new Error("Sim kinda crazy ngl ngl, Harry you figure this one out lol");
  }

  public double getAngle2() {
    // TODO: Sim kinda crazy ngl ngl, Harry you figure this one out lol
    throw new Error("Sim kinda crazy ngl ngl, Harry you figure this one out lol");
  }

  @Override
  public double getVelocity1() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity1'");
  }

  @Override
  public double getVelocity2() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getVelocity2'");
  }
}

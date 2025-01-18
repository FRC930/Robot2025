package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeIOSim implements IntakeIO {
  public MotionMagicVoltage Request;
  /** J3_KP makes the joint motor know what speed to run at */
  public final double J3_KP = 1.0;

  public final double J3_KD = 1.0;
  public final double J3_GEARING = 1.0;

  private final DCMotorSim jointSim;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private TalonFX talon;

  private static final double flwheel_kA = 0.00032;
  private static final double flwheel_kV = 1.0;

  private final LinearSystem<N2, N1, N2> m_flywheelPlant =
      LinearSystemId.createDCMotorSystem(flwheel_kV, flwheel_kA);

  public IntakeIOSim(int motor3Id) {
    jointSim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.1, 0.1);
    talon = new TalonFX(motor3Id, "rio");
    Request = new MotionMagicVoltage(0);
  }

  @Override
  public void setSpeed(double speed) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setSpeed'");
  }

  @Override
  public Trigger intakeSensorBro() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'intakeSensorBro'");
  }
}

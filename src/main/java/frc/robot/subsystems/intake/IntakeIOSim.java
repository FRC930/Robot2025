package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOSim implements IntakeIO {
  /** J3_KP makes the joint motor know what speed to run at */
  public final double J3_KP = 1.0;

  public final double J3_KD = 1.0;
  public final double J3_GEARING = 1.0;

  private Voltage appliedVoltage = Volts.of(0.0);

  private final DCMotorSim sim;

  private final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private static final double flwheel_kA = 0.1;
  private static final double flwheel_kV = 1.0;

  private final LinearSystem<N2, N1, N2> m_flywheelPlant = LinearSystemId.createDCMotorSystem(flwheel_kV, flwheel_kA);

  private LoggedTunableNumber intakeSensorSim;
  private MutDistance intakeSensorDistance;

  public IntakeIOSim(int motor3Id) {
    sim = new DCMotorSim(m_flywheelPlant, DRIVE_GEARBOX, 0.1, 0.1);
    intakeSensorDistance = Meters.mutable(1);
    intakeSensorSim = new LoggedTunableNumber("RobotState/Intake/SensorInput", 1);
  }

  @Override
  public void setTarget(Voltage setpoint) {
    this.appliedVoltage = setpoint;
  }

  @Override
  public void updateInputs(IntakeInputs input) {
    //Inputs
    intakeSensorDistance.mut_replace(Meters.of(intakeSensorSim.get()));

    //Logging
    input.angularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.statorCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);
    input.voltage.mut_replace(Volts.of(sim.getInputVoltage()));
    input.sensorDistance.mut_replace(intakeSensorDistance);

    // Periodic
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  @Override
  public Supplier<Distance> intakeSensorBro() {
    return () -> {
      return intakeSensorDistance.copy();
    };
  }

  @Override
  public void stop() {
  }
}

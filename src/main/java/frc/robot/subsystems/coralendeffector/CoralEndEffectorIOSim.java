package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.LoggedTunableNumber;

public class CoralEndEffectorIOSim implements CoralEndEffectorIO {
  private Voltage appliedVoltage = Volts.mutable(0.0);
  private LoggedTunableNumber coralEESensorSim = new LoggedTunableNumber("Coral End Effector/SensorDistanceInches", 1);

  private final FlywheelSim sim;

  public CoralEndEffectorIOSim(int motorId) {
    sim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60Foc(1), 
        0.0005, 
        1
        ), 
      DCMotor.getKrakenX60Foc(1), 0.01);
  }

  @Override
  public void setTarget(Voltage target) {
    runVolts(target);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void updateInputs(CoralEndEffectorInputs input) {
    input.angularVelocity.mut_replace(sim.getAngularVelocity());
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);
    input.hasCoral = Inches.of(coralEESensorSim.get()).lt(Inches.of(CoralEndEffector.CORAL_DISTANCE_THRESHOLD.get()));

    // Periodic
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  @Override
  public void stop() {
    setTarget(Volts.of(0.0));
  }
  
}

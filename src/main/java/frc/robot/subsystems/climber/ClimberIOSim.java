package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ClimberIOSim implements ClimberIO {

  private Voltage appliedVoltage = Volts.mutable(0.0);

  private Angle servoTarget = Angle.ofRelativeUnits(0, Degrees);

  private final FlywheelSim sim;

  public ClimberIOSim(int motorId) {
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

  @Override
  public void setServoTarget(Angle angle) {
    System.out.println("CLIMBER SERVO SERVING AT ANGLE " + angle.in(Degrees));
    servoTarget = angle;
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }

  @Override
  public void updateInputs(ClimberInputs input) {
    input.angularVelocity.mut_replace(
        DegreesPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond),
        DegreesPerSecond);
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);
    input.servoTarget.mut_replace(servoTarget);
    input.servoPos.mut_replace(servoTarget); // TODO Sim is perfect and has instant servos!

    // Periodic
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  @Override
  public void stop() {
    setTarget(Volts.of(0.0));
  }
  
}

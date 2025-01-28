package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorFeedforward ff = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
  private final ProfiledPIDController controller = new ProfiledPIDController(0.4, 0.0, 0.0, new Constraints(100000, 361));
  private final ElevatorSim sim;

  private Voltage appliedVoltage = Volts.mutable(0.0);

  public ElevatorIOSim(int motorId, ElevatorSim elevatorSim) {
    sim = elevatorSim;
  }

  @Override
  public void setTarget(Distance target) {
    controller.setGoal(new State(target.in(Meters), 0));
  }

  private void updateVoltageSetpoint() {
    Distance currentPosition = Meters.of(sim.getPositionMeters());

    Voltage controllerVoltage = Volts.of(controller.calculate(currentPosition.in(Meters)));
    Voltage feedForwardVoltage =
          Volts.of(
              ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    this.appliedVoltage = volts;
  }
  
  @Override
  public void updateInputs(ElevatorInputs input) {
    input.distance.mut_replace(Distance.ofRelativeUnits(sim.getPositionMeters(), Meters));
    input.velocity.mut_replace(MetersPerSecond.of(sim.getVelocityMetersPerSecond()));
    input.setPoint.mut_replace(Meters.of(controller.getGoal().position));
    input.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(appliedVoltage);

    // Periodic
    updateVoltageSetpoint();
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);
  }

  @Override
  public void stop() {
    Distance currentDistance = Distance.ofRelativeUnits(0, Meters);
    controller.reset(currentDistance.in(Meters));
    runVolts(Volts.of(0));
  }
}

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.Gains;

/**
 * An implementation of the {@link ElevatorIO} for running the robot in simulation.
 * @see {@link ElevatorIO}
 * {@inheritDoc}
 */
public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(0.0, 0.8, 0.0, 0.0);
  private final ProfiledPIDController m_controller = new ProfiledPIDController(5.0, 0.0, 0.0, new Constraints(90, 120));
  private final ElevatorSim m_sim;

  private Distance m_target = Inches.of(0);
  private final MutVoltage m_appliedVoltage = Volts.mutable(0.0);

  /**
   * A constructor to create a {@link ElevatorIOSim} with a specified motorID.
   * @param elevatorSim The WPILib elevator simulation. @see {@link ElevatorSim}
   */
  public ElevatorIOSim(ElevatorSim elevatorSim) {
    m_sim = elevatorSim;
  }

  @Override
  public void setTarget(Distance target) {
    this.m_target = target;
    m_controller.setGoal(target.in(Inches));
  }

  private void updateVoltageSetpoint() {
    Distance currentPosition = Meters.of(m_sim.getPositionMeters());
    LinearVelocity currentVelocity = MetersPerSecond.of(m_sim.getVelocityMetersPerSecond());
    Voltage controllerVoltage = Volts.of(m_controller.calculate(currentPosition.in(Inches), this.m_target.in(Inches)));
    Voltage feedForwardVoltage = Volts.of(m_feedforward.calculate(currentVelocity.in(InchesPerSecond)));

    Voltage effort = controllerVoltage.plus(feedForwardVoltage);

    runVolts(effort);
  }

  private void runVolts(Voltage volts) {
    double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
    m_appliedVoltage.mut_replace(clampedEffort, Volts);
    m_sim.setInputVoltage(clampedEffort);
  }
  
  @Override
  public void updateInputs(ElevatorInputs input) {
    m_sim.update(0.02);
    input.distance.mut_replace(m_sim.getPositionMeters(), Meters);
    input.velocity.mut_replace(MetersPerSecond.of(m_sim.getVelocityMetersPerSecond()));
    input.setPoint.mut_replace(Inches.of(m_controller.getGoal().position));
    input.supplyCurrent.mut_replace(m_sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageSetPoint.mut_replace(m_appliedVoltage);

    // Periodic
    updateVoltageSetpoint();
  }

  @Override
  public void stop() {
    Distance currentDistance = Distance.ofRelativeUnits(0, Meters);
    m_controller.reset(currentDistance.in(Meters));
    runVolts(Volts.of(0));
  }

  public void setGains(Gains gains) {
    // DriverStation.reportWarning("Sim gains tuning not implemented", true);
  }
}

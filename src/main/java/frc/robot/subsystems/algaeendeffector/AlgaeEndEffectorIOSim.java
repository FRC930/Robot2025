package frc.robot.subsystems.algaeendeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.LoggedTunableNumber;

/**
 * An implementation of the {@link AlgaeEndEffectorIO} for running the robot in simulation.
 * @see {@link AlgaeEndEffectorIO}
 * {@inheritDoc}
 */
public class AlgaeEndEffectorIOSim implements AlgaeEndEffectorIO {
  private Voltage m_appliedVoltage = Volts.mutable(0.0);
  private final LoggedTunableNumber m_algaeEESensorSim = new LoggedTunableNumber("Sensors/AlgaeEndEffector/SensorDistanceInches", 1);

  private final FlywheelSim m_sim;

  /**
   * A constructor to create a {@link AlgaeEndEffectorIOSim} with a specified motorID.
   * @param motorId The simulated ID of the motor.
   */
  public AlgaeEndEffectorIOSim(int motorId) {
    m_sim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60Foc(1), 
        0.0005, 
        1
        ), 
      DCMotor.getKrakenX60Foc(1), 0.01);
    Meters.mutable(1);
  }

  /**
   * @see {@link AlgaeEndEffectorIO#setTarget}
   * {@inheritDoc}
   */
  @Override
  public void setTarget(Voltage volts) {
    this.m_appliedVoltage = volts;
  }

  /**
   * @see {@link AlgaeEndEffectorIO#updateInputs}
   * {@inheritDoc}
   */
  @Override
  public void updateInputs(AlgaeEndEffectorInputs input) {
    input.angularVelocity.mut_replace(m_sim.getAngularVelocity());
    input.supplyCurrent.mut_replace(m_sim.getCurrentDrawAmps(), Amps);
    input.torqueCurrent.mut_replace(input.supplyCurrent.in(Amps), Amps);
    input.voltageTarget.mut_replace(m_appliedVoltage);
    input.algaeDistance.mut_replace(Inches.of(m_algaeEESensorSim.get()));

    // Periodic
    m_sim.setInputVoltage(m_appliedVoltage.in(Volts));
    m_sim.update(0.02);
  }

  /**
   * @see {@link AlgaeEndEffectorIO#stop}
   * {@inheritDoc}
   */
  @Override
  public void stop() {
    setTarget(Volts.of(0.0));
  }
}

package frc.robot.subsystems.algaeendeffector;

import com.ctre.phoenix6.hardware.CANrange;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.MotorType;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanDef;

/**
 * An implementation of the {@link AlgaeEndEffectorIO} for Nova motor controllers.
 * @see {@link AlgaeEndEffectorIO}
 * {@inheritDoc}
 */
public class AlgaeEndEffectorIONova implements AlgaeEndEffectorIO {
  private final ThriftyNova m_motor;
  private final CANrange m_sensor;

  private Voltage m_setPoint = Volts.of(0);

  /**
   * A constructor to create a {@link AlgaeEndEffectorIONova} with specified CAN definitions.
   * @param motorCanDef The CAN definition for the Thrifty Nova Minion motors.
   * @param sensorCanDef The CAN definition for the CANrange sensors.
   */
  public AlgaeEndEffectorIONova(CanDef motorCanDef, CanDef sensorCanDef) {
    m_motor = new ThriftyNova(motorCanDef.id()).setMotorType(MotorType.MINION);
    m_sensor = new CANrange(sensorCanDef.id(),sensorCanDef.bus());
  }

  /**
   * @see {@link AlgaeEndEffectorIO#updateInputs}
   * {@inheritDoc}
   */
  @Override
  public void updateInputs(AlgaeEndEffectorInputs inputs) {
    inputs.angularVelocity.mut_replace(m_motor.getVelocity(), RadiansPerSecond);
    inputs.voltageTarget.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(m_motor.getVoltage(), Volts);
    inputs.supplyCurrent.mut_replace(m_motor.getSupplyCurrent(), Amps);
    inputs.torqueCurrent.mut_replace(m_motor.getStatorCurrent(), Amps);
    inputs.algaeDistance.mut_replace(m_sensor.getDistance().getValue());
  }

  /**
   * @see {@link AlgaeEndEffectorIO#setTarget}
   * {@inheritDoc}
   */
  @Override
  public void setTarget(Voltage target) {
    m_motor.setVoltage(target);
    m_setPoint = target;
  }

  /**
   * @see {@link AlgaeEndEffectorIO#stop}
   * {@inheritDoc}
   */
  @Override
  public void stop() {
    m_motor.setVoltage(0.0);
  }
}

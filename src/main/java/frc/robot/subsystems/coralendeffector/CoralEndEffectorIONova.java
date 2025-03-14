package frc.robot.subsystems.coralendeffector;

import com.thethriftybot.ThriftyNova;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.util.CanDef;



public class CoralEndEffectorIONova implements CoralEndEffectorIO {
  public ThriftyNova Motor;
  public ArmInputs inputs;

  private CANrange m_sensor;

  private Voltage m_setPoint = Voltage.ofBaseUnits(0, Volts);

  public CoralEndEffectorIONova(CanDef motorCanDef, CanDef sensorCanDef) {
    Motor = new ThriftyNova(motorCanDef.id());
    m_sensor = new CANrange(sensorCanDef.id(),sensorCanDef.bus());
  }

  @Override
  public void updateInputs(FingeysInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity(), RadiansPerSecond);
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(Motor.getVoltage(), Volts);
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent(), Amps);
  }

  @Override
  public void setTarget(Voltage target) {
    Motor.setVoltage(target);;
    m_setPoint = target;
  }

  @Override
  public void stop() {
    Motor.setVoltage(0.0);
  }

  @Override
  public Distance getDistance() {
    return m_sensor.getDistance().getValue();

  }
}

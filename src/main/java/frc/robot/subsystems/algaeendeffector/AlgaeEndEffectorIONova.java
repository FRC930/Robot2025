package frc.robot.subsystems.algaeendeffector;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.MotorType;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.ArmJointIO.ArmInputs;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.util.CanDef;



public class AlgaeEndEffectorIONova implements AlgaeEndEffectorIO {
  private ThriftyNova motor;
  private CANrange m_sensor;

  private Voltage m_setPoint = Volts.of(0);

  public AlgaeEndEffectorIONova(CanDef motorCanDef, CanDef sensorCanDef) {
    motor = new ThriftyNova(motorCanDef.id()).setMotorType(MotorType.MINION);
    m_sensor = new CANrange(sensorCanDef.id(),sensorCanDef.bus());
  }

  @Override
  public void updateInputs(AlgaeEndEffectorInputs inputs) {
    inputs.angularVelocity.mut_replace(motor.getVelocity(), RadiansPerSecond);
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(motor.getVoltage(), Volts);
    inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent(), Amps);
    inputs.torqueCurrent.mut_replace(motor.getStatorCurrent(), Amps);
    inputs.hasAlgae = m_sensor.getDistance().getValue().lt(Inches.of(AlgaeEndEffector.ALGAE_DISTANCE_THRESHOLD.get()));
  }

  @Override
  public void setTarget(Voltage target) {
    motor.setVoltage(target);;
    m_setPoint = target;
  }

  @Override
  public void stop() {
    motor.setVoltage(0.0);
  }
  
}

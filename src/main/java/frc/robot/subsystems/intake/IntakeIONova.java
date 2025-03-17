package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.MotorType;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class IntakeIONova implements IntakeIO {
  public VoltageOut Request;
  public ThriftyNova Motor;

  private CANrange rangeSensor;
  private Voltage m_setPoint = Voltage.ofBaseUnits(0, Volts);

  public IntakeIONova(CanDef motorCanDef, CanDef sensorCanDef) {
    Motor = new ThriftyNova(motorCanDef.id()).setMotorType(MotorType.MINION);
    rangeSensor = new CANrange(sensorCanDef.id(),sensorCanDef.bus());
    Request = new VoltageOut(0.0);

    configureTalons();
  }

  private void configureTalons() {
    Motor.setMaxCurrent(CurrentType.STATOR, 80);
    Motor.setMaxCurrent(CurrentType.SUPPLY,30);
    Motor.setInversion(true);
    Motor.setBrakeMode(true);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.angularVelocity.mut_replace(RadiansPerSecond.of(Motor.getVelocity()));
    inputs.voltageSetPoint.mut_replace(m_setPoint);
    inputs.voltage.mut_replace(Volts.of(Motor.getVoltage()));
    inputs.supplyCurrent.mut_replace(Amps.of(Motor.getSupplyCurrent()));
    inputs.hasCoral = rangeSensor.getDistance().getValue().lt(Inches.of(Intake.CORAL_DISTANCE_THRESHOLD.get()));
  }

  @Override
  public void setTarget(Voltage target) {
    Motor.setVoltage(target);
    m_setPoint = target;
  }

  @Override
  public void stop() {
    Motor.setVoltage(0);
  }
}

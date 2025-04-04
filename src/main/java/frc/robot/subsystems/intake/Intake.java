package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase{
  public static LoggedTunableNumber CORAL_DISTANCE_THRESHOLD = new LoggedTunableNumber("Intake/SENSORTHRESHOLD", 3.75);

  private IntakeIO m_intakeIO;
  private IntakeInputsAutoLogged loggedIntake = new IntakeInputsAutoLogged();
  private Debouncer debouncer = new Debouncer(0.1,DebounceType.kBoth);


  private static final Distance LOWER_SENSOR_BOUND = Inches.of(3.0);
  private static final Distance UPPER_SENSOR_BOUND = Inches.of(3.3);


  public Intake(IntakeIO intakeIO) {
    m_intakeIO = intakeIO;
    loggedIntake.angularVelocity = DegreesPerSecond.mutable(0);
    loggedIntake.supplyCurrent = Amps.mutable(0);
    loggedIntake.statorCurrent = Amps.mutable(0);
    loggedIntake.voltage = Volts.mutable(0);
    loggedIntake.voltageSetPoint = Volts.mutable(0);
    loggedIntake.coralDistance = Inches.mutable(100);

    Logger.recordOutput("RobotState/Intake/LowerSensorBound",LOWER_SENSOR_BOUND);
    Logger.recordOutput("RobotState/Intake/UpperSensorBound",UPPER_SENSOR_BOUND);
  }

  private void setTarget(Voltage target) {
    m_intakeIO.setTarget(target);
  }

  public Command getNewSetVoltsCommand(LoggedTunableNumber volts) {
    return new InstantCommand(() -> {
      setTarget(Volts.of((volts.get())));
    }, this);
  }

  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(() -> {
      setTarget(Volts.of(i));
    }, this);
  }

  public Command getNewSetSpeedCommand(double percentOutput) {
    return new InstantCommand(() -> {
      double volts = 12.0 * percentOutput;
      setTarget(Volts.of(volts));
    }, this);
  }

  public Trigger hasCoralTrigger() {
    return new Trigger(() -> 
      loggedIntake.coralDistance.gt(LOWER_SENSOR_BOUND) && loggedIntake.coralDistance.lt(UPPER_SENSOR_BOUND)
    ).debounce(0.1);
  }

  @Override
  public void periodic() {
    m_intakeIO.updateInputs(loggedIntake);
    Logger.processInputs("RobotState/Intake", loggedIntake);
  }
}

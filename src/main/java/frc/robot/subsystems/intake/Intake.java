package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO;

public class Intake extends SubsystemBase{
  private IntakeIO m_intakeIO;

  private Voltage setpoint;

  IntakeInputsAutoLogged loggedIntake = new IntakeInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    m_intakeIO = intakeIO;
    loggedIntake.angularVelocity = DegreesPerSecond.mutable(0);
    loggedIntake.supplyCurrent = Amps.mutable(0);
    loggedIntake.statorCurrent = Amps.mutable(0);
    loggedIntake.sensorDistance = Meters.mutable(0);
    loggedIntake.voltage = Volts.mutable(0);
    loggedIntake.voltageSetPoint = Volts.mutable(0);
    
    loggedIntake.timestamp = 0.0;
  }

  public void setTarget(Voltage target) {
    setpoint = target;
    m_intakeIO.setTarget(target);
  }

  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(i));
        },
        this);
  }

  @Override
  public void periodic() {
    m_intakeIO.updateInputs(loggedIntake);
    Logger.processInputs("RobotState/Intake", loggedIntake);
  }
}

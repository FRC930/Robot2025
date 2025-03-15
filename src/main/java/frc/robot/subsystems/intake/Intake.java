package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import javax.sound.sampled.ReverbType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase{
  private IntakeIO m_intakeIO;

  IntakeInputsAutoLogged loggedIntake = new IntakeInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    m_intakeIO = intakeIO;
    loggedIntake.angularVelocity = DegreesPerSecond.mutable(0);
    loggedIntake.supplyCurrent = Amps.mutable(0);
    loggedIntake.statorCurrent = Amps.mutable(0);
    loggedIntake.voltage = Volts.mutable(0);
    loggedIntake.voltageSetPoint = Volts.mutable(0);
  }

  public void setTarget(Voltage target) {
    m_intakeIO.setTarget(target);
  }

  public Command getNewSetVoltsCommand(LoggedTunableNumber volts) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of((volts.get())));
        },
        this);
  }

  public Command getNewSetVoltsCommand(double i) {
    return new InstantCommand(
        () -> {
          setTarget(Volts.of(i));
        },
        this);
  }

  public Command getNewSetSpeedCommand(double percentOutput) {
    return new InstantCommand(
      () -> {
        double volts = 12.0 * percentOutput;
        setTarget(Volts.of(volts));
      },
      this
    );
  }

  public Trigger getNewHasCoralTrigger() {
    return new Trigger(() -> {
      return true;
    });
  }

  @Override
  public void periodic() {
    m_intakeIO.updateInputs(loggedIntake);
    Logger.processInputs("RobotState/Intake", loggedIntake);
  }
}

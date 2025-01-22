package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorIO m_ElevatorIO;
  private Angle setpoint;
  private String loggerSuffix;

  ElevatorInputsAutoLogged loggedelevator = new ElevatorInputsAutoLogged();

  public ElevatorSubsystem(ElevatorIO elevatorIO, Optional<String> loggedName) {
    //Keegan - Turn this into Distance
    m_ElevatorIO = elevatorIO;
    loggedelevator.jointAngle = Degrees.mutable(0);
    loggedelevator.jointAngularVelocity = DegreesPerSecond.mutable(0);
    loggedelevator.jointSetPoint = Degrees.mutable(0);
    loggedelevator.supplyCurrent = Amps.mutable(0);
    loggedelevator.timestamp = 0.0;
    loggedelevator.torqueCurrent = Amps.mutable(0);
    loggedelevator.voltageSetPoint = Volts.mutable(0);

    if (loggedName.isPresent()) {
      loggerSuffix = loggedName.get();
    } else {
      loggerSuffix = "" + this.hashCode();
    }
  }

  public void setAngle(Angle angle) {
    setpoint = angle;
    m_ElevatorIO.setTarget(null, angle);
  }

  public Command getNewSetAngleCommand(double i) {
    return new InstantCommand(
        () -> {
          setAngle(Degrees.of(i));
        },
        this);
  }

  @Override
  public void periodic() {
    m_ElevatorIO.updateInputs(loggedelevator);
    Logger.processInputs("RobotState/Elevator" + loggerSuffix, loggedelevator);
  }
}

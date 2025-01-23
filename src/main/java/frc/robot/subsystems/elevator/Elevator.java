package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private ElevatorIO m_ElevatorIO;
  private String loggerSuffix;

  ElevatorInputsAutoLogged loggedelevator = new ElevatorInputsAutoLogged();

  public Elevator(ElevatorIO elevatorIO, Optional<String> loggedName) {
    //Keegan - Turn this into Distance
    m_ElevatorIO = elevatorIO;
    loggedelevator.elevatorDistance = Meters.mutable(0);
    loggedelevator.elevatorVelocity = MetersPerSecond.mutable(0);
    loggedelevator.elevatorSetPoint = Meters.mutable(0);
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

  public void setDistance(Distance target) {
    m_ElevatorIO.setTarget(target);
  }

  public Command getNewSetDistanceCommand(double i) {
    return new InstantCommand(
        () -> {
          setDistance(Meters.of(i));
        },
        this);
  }

  @Override
  public void periodic() {
    m_ElevatorIO.updateInputs(loggedelevator);
    Logger.processInputs("RobotState/Elevator" + loggerSuffix, loggedelevator);
  }
}

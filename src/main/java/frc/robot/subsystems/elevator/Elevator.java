package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Elevator extends SubsystemBase {
  private ElevatorIO m_ElevatorIO;

  ElevatorInputsAutoLogged loggedelevator = new ElevatorInputsAutoLogged();

  public Elevator(ElevatorIO elevatorIO) {
    m_ElevatorIO = elevatorIO;
    loggedelevator.distance = Meters.mutable(0);
    loggedelevator.velocity = MetersPerSecond.mutable(0);
    loggedelevator.setPoint = Meters.mutable(0);
    loggedelevator.supplyCurrent = Amps.mutable(0);
    loggedelevator.torqueCurrent = Amps.mutable(0);
    loggedelevator.voltageSetPoint = Volts.mutable(0);

    RobotState.instance().setElevatorSource(loggedelevator.distance);
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
    Logger.processInputs("RobotState/Elevator", loggedelevator);
  }
}

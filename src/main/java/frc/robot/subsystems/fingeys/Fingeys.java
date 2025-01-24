package frc.robot.subsystems.fingeys;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmInputsAutoLogged;
import frc.robot.subsystems.arm.constants.ArmJointConstants;

public class Fingeys extends SubsystemBase {
  private FingeysIO m_FingeysIO;

  private AngularVelocity setpoint;
  private String loggerSuffix;

  FingeysInputsAutoLogged loggedfingeys = new FingeysInputsAutoLogged();

  private final FingeysConstants m_Constants;

  public Fingeys(FingeysIO fingeysIO) {
    m_FingeysIO = fingeysIO;
    loggedfingeys.angularVelocity = DegreesPerSecond.mutable(0);
    loggedfingeys.supplyCurrent = Amps.mutable(0);
    loggedfingeys.timestamp = 0.0;
    loggedfingeys.torqueCurrent = Amps.mutable(0);
    loggedfingeys.voltageSetPoint = Volts.mutable(0);

    m_Constants = m_FingeysIO.getConstants();

    loggerSuffix = m_Constants.LoggedName;
  }

  public void setAngularVelocity(AngularVelocity angularVelocity) {
    setpoint = angularVelocity;
    m_FingeysIO.setTarget(angularVelocity);
  }

  public Command getNewSetAngleCommand(double i) {
    return new InstantCommand(
        () -> {
          setAngularVelocity(DegreesPerSecond.of(i));
        },
        this);
  }

  @Override
  public void periodic() {
    m_FingeysIO.updateInputs(loggedfingeys);
    Logger.processInputs("RobotState/" + loggerSuffix, loggedfingeys);
  }
}

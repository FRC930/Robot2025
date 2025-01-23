package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.constants.ArmJointConstants;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ArmJoint extends SubsystemBase {
  private ArmJointIO m_ArmIO;
  private Angle setpoint;
  private String loggerSuffix;

  ArmInputsAutoLogged loggedarm = new ArmInputsAutoLogged();

  private final ArmJointConstants m_Constants;

  public ArmJoint(ArmJointIO armIO, ArmJointConstants constants) {
    m_ArmIO = armIO;
    loggedarm.jointAngle = Degrees.mutable(0);
    loggedarm.jointAngularVelocity = DegreesPerSecond.mutable(0);
    loggedarm.jointSetPoint = Degrees.mutable(0);
    loggedarm.supplyCurrent = Amps.mutable(0);
    loggedarm.timestamp = 0.0;
    loggedarm.torqueCurrent = Amps.mutable(0);
    loggedarm.voltageSetPoint = Volts.mutable(0);

    m_Constants = armIO.getConstants();
    
    loggerSuffix = m_Constants.LoggedName;
  }

  public void setAngle(Angle angle) {
    setpoint = angle;
    m_ArmIO.setTarget(angle);
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
    m_ArmIO.updateInputs(loggedarm);
    Logger.processInputs("RobotState/ArmJoint" + loggerSuffix, loggedarm);
  }
}

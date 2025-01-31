
package frc.robot.subsystems.fingeys;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Fingeys extends SubsystemBase {
  private FingeysIO m_FingeysIO;

  FingeysInputsAutoLogged loggedfingeys = new FingeysInputsAutoLogged();

  public Fingeys(FingeysIO fingeysIO) {
    m_FingeysIO = fingeysIO;
    loggedfingeys.angularVelocity = DegreesPerSecond.mutable(0);
    loggedfingeys.supplyCurrent = Amps.mutable(0);
    loggedfingeys.torqueCurrent = Amps.mutable(0);
    loggedfingeys.voltageSetPoint = Volts.mutable(0);
    loggedfingeys.voltage = Volts.mutable(0);
  }

  public void setTarget(Voltage target) {
    m_FingeysIO.setTarget(target);
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

  @Override
  public void periodic() {
    m_FingeysIO.updateInputs(loggedfingeys);
    Logger.processInputs("RobotState/Fingeys", loggedfingeys);
  }
}

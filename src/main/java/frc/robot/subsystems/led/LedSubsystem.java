package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class LedSubsystem extends SubsystemBase {
  private LedIO LEDs;
  private Map<String, AddressableLEDBufferView> LEDViews;
  private ArrayList<String> errors = new ArrayList<>();

  public LedSubsystem(LedIO io) {
    this.LEDs = io;
    LEDViews = new HashMap<String, AddressableLEDBufferView>();
  }

  public void addLEDView(String name, int start, int end) {
    try {
      AddressableLEDBufferView view = LEDs.get().createView(start, end);
      LEDViews.put(name, view);
    } catch (Exception e) {
      DriverStation.reportWarning("LED View could not be initialized! : %s \n".formatted(e), true);
      errors.add("LED View could not be initialized!");
    }
  }

  private AddressableLEDBufferView getView(String name) {
    try {
      AddressableLEDBufferView view = LEDViews.get(name);
      return view;
    } catch (Exception e) {
      DriverStation.reportWarning("LED View could not be found! : %s \n".formatted(e), true);
      errors.add("LED View could not be found!");
    }
    return null;
  }

  @Override
  public void periodic() {
    LEDs.periodic();
    Logger.recordOutput("LEDSubsystems/TEST/ErrorCodes", errors.toArray(new String[errors.size()]));
  }

  public Command newRunPatternCommand(String targetView, LEDPattern pattern) {
    return new InstantCommand(
        () -> {
          try {
            pattern.applyTo(getView(targetView));
          } catch (Exception e) {
            DriverStation.reportWarning("LED View could not be found! : %s \n".formatted(e), true);
            errors.add("LED View could not be found!");
          }
        },
        this);
  }
}

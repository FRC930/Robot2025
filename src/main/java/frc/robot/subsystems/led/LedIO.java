package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LedIO {
  public AddressableLEDBuffer get();

  public void periodic();
}

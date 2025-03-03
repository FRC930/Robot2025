package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedIOReal implements LedIO {

  private AddressableLED LEDs;
  private AddressableLEDBuffer buffer;

  public LedIOReal(int PWMPort, int bufferLength) {
    LEDs = new AddressableLED(PWMPort);
    buffer = new AddressableLEDBuffer(bufferLength);
    LEDs.setLength(bufferLength);
    LEDs.setData(buffer);
    LEDs.start();
  }

  @Override
  public AddressableLEDBuffer get() {
    return buffer;
  }

  @Override
  public void periodic() {
    LEDs.setData(buffer);
  }
}

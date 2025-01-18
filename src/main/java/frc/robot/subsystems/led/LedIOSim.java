package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class LedIOSim implements LedIO {

  private AddressableLEDSim LEDSim;
  private AddressableLED LEDs;

  private AddressableLEDBuffer buffer;

  public LedIOSim(int PWMPort, int bufferLength) {
    LEDs = new AddressableLED(PWMPort);
    LEDSim = new AddressableLEDSim(LEDs);
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

// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static StatusCode tryUntilOk(
      int maxAttempts, Supplier<StatusCode> command, Optional<ParentDevice> device) {
    StatusCode error = StatusCode.NotFound;
    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();
      if (error.isOK()) break;
      DriverStation.reportWarning(
          String.format(
              "Unable to configure device %s: %s",
              device.isPresent() ? device.get().getDeviceID() : "?", error.toString()),
          true);
    }
    return error;
  }

      /**
     * Get position from controller so dont have to explicitly cast applied controller (could have had runtime casting error when switch controllers)
     * @param motor
     * @param defaultPosition
     * @return
     */
  public static double getPositionFromController(ParentDevice motor, double defaultPosition) {
    // Position configuration may not be available yet, so allow for Position not being available yet
    Map<String, String> map = motor.getAppliedControl().getControlInfo();
    String positionString = map.get("Position");
    double position= defaultPosition; // If controller is not available delayed configurations
    if(positionString != null) {
        position = Double.valueOf(positionString);
    }
    return position;
      
    }

  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    tryUntilOk(maxAttempts, command, Optional.empty());
  }
}

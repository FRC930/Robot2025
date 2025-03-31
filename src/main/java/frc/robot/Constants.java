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

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //A flag in case our practice field breaks the logged tunable numbers. Forces loggedtunablnumbers to be enabled
  public static final boolean overrideEnableLoggedTunableNumbers = false;

  public static final boolean isDSConnected = DriverStation.waitForDsConnection(1);
  public static final boolean isInElim = DriverStation.getMatchType() == MatchType.Elimination;
  public static final boolean isInQual = DriverStation.getMatchType() == MatchType.Qualification;
  
  public static final boolean isInMatch = isInElim || isInQual;
  
  // When changing Mode.SIM to Mode.REPLAY, also change SimGui default flag in build.gradle
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = !isInMatch || overrideEnableLoggedTunableNumbers;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}

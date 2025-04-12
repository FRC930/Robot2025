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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // Camera names, must match names configured on coprocessor
  public static String limelightLeftName = "limelight-backl";
  public static String limelightRightName = "limelight-backr";
  public static String limelightFrontName = "limelight-front";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCameraLeft =
  new Transform3d(
    //IP 10.9.30.16
    //back Left (on endgame) values on robot, 
    // LL Forward -0.236449(-9.309 in y val from cad), LL Right -0.190119(-7.485 in x val from cad), LL up 0.317932(-12.517 in), LL Roll 0, LL Pitch -10, LL Yaw -172.5
    //LL RIGHT IS -, IN CODE(y value) IT'S +
      -0.236449,
      0.190119, 
      0.317932,
      //LL PITCH IS +, IN CODE IT'S -
      new Rotation3d(0.0, Math.toRadians(-10.0), Math.toRadians(-172.5)));
public static Transform3d robotToCameraRight =
  new Transform3d(
    // 10.9.30.15
    //back right values on robot, 
    // LL Forward -0.236449(9.309in y val from cad), LL Right 0.190119(-7.485in x val from cad), LL up 0.318008(12.52in), LL Roll 0, LL Pitch 10, LL Yaw 172.5.
    //LL RIGHT IS +, IN CODE(y value) IT'S -
      -0.236449,
      -0.190119,
      0.318008,
      //LL PITCH IS +, IN CODE IT'S -
      new Rotation3d(0.0, Math.toRadians(-10.0), Math.toRadians(172.5)));

      // front cam values on robot, 
      // LL Forward 0.391, LL Right 0.0, LL up 0.198, LL Roll 0.0, LL Pitch 0.0, LL Yaw 0.0.
      public static Transform3d robotToCameraFront =
      new Transform3d(
          .0,
          -0.391,
          0.198,
          new Rotation3d(0.0, Math.toRadians(0.0), Math.toRadians(0.0)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines(0.01), for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 1.0; // Meters, 0.01 was original value was hyper sensitive.
  public static double angularStdDevBaseline = 0.06;

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}

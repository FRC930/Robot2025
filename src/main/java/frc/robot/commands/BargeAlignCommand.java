package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;

public class BargeAlignCommand extends AutoAlignCommand {
    private static final AprilTagFieldLayout aprilTagLayout = Drive.getAprilTagLayout();
    public BargeAlignCommand(Drive drive) {
        super((p)->getBargeScorePose(p),drive);
    }

    public static Pose2d getBargeScorePose(Pose2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        // Default bounds for upper side
        double upperYBound = 0.0;
        double lowerYBound = 0.0;
        double xSetpoint = 0.0;
        double rotation = 0.0;
        switch (alliance) {
        case Red:
        // Bounds for lower side
        upperYBound = 3.5;
        lowerYBound = 0.5;
        xSetpoint = 10.0;
            break;
        case Blue:  
        // Bounds for upper side
        upperYBound = 7.5;
        lowerYBound = 4.5;
        xSetpoint = 7.5;
            break;
        }

        return new Pose2d(xSetpoint,MathUtil.clamp(robotPose.getY(), lowerYBound, upperYBound),new Rotation2d(rotation));
  }
}

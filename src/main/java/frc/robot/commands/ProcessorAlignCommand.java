package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;

public class ProcessorAlignCommand extends AutoAlignCommand {
    private static final AprilTagFieldLayout aprilTagLayout = Drive.getAprilTagLayout();
    public ProcessorAlignCommand(Drive drive) {
        super((p)->getProcessorScorePose(p),drive);
    }

    public static Pose2d getProcessorScorePose(Pose2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Transform2d transform = new Transform2d(0.68,0.0,Rotation2d.kZero);
        Pose2d target = Pose2d.kZero;

        switch (alliance) {
        case Red:
        target = aprilTagLayout.getTagPose(3).get().toPose2d();
            break;
        case Blue: 
        target = aprilTagLayout.getTagPose(18).get().toPose2d();
            break;
        }

        return target.transformBy(transform);
  }
}

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class ProcessorAlignCommand extends AutoAlignCommand {
    private static final AprilTagFieldLayout aprilTagLayout = Drive.getAprilTagLayout();

    private static final LoggedTunableNumber offsetB = new LoggedTunableNumber("AutoAlignCommands/ProcessorAlignCommand/offsetB",0.68);
    private static final LoggedTunableNumber offsetR = new LoggedTunableNumber("AutoAlignCommands/ProcessorAlignCommand/offsetR",0.0);

    public ProcessorAlignCommand(Drive drive) {
        super((p)->getProcessorScorePose(p),drive);
    }

    public static Pose2d getProcessorScorePose(Pose2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Transform2d transform = new Transform2d(offsetB.get(),offsetR.get(),Rotation2d.kZero);
        Pose2d target = Pose2d.kZero;

        switch (alliance) {
        case Red:
        target = aprilTagLayout.getTagPose(3).get().toPose2d();
            break;
        case Blue: 
        target = aprilTagLayout.getTagPose(16).get().toPose2d();
            break;
        }

        return target.transformBy(transform);
  }
}

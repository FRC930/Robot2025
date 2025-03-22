package frc.robot.commands;

import java.io.Console;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class BargeAlignCommand extends AutoAlignCommand {
    private static final AprilTagFieldLayout aprilTagLayout = Drive.getAprilTagLayout();
    private static final LoggedTunableNumber offsetB = new LoggedTunableNumber("BargeAlignCommand/offsetB",0.5);
    private static final LoggedTunableNumber maxHorziontalOffset = new LoggedTunableNumber("BargeAlignCommand/maxHorizontalOffset", 1.5);

    public BargeAlignCommand(Drive drive, Supplier<Double> strafeControl) {
        super((p)->getBargeScorePose(p),drive);
    }

    public static Pose2d getBargeScorePose(Pose2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Pose2d originPose = alliance == Alliance.Red ? aprilTagLayout.getTagPose(5).orElse(Pose3d.kZero).toPose2d() : aprilTagLayout.getTagPose(14).orElse(Pose3d.kZero).toPose2d();
        double xOffset = alliance == Alliance.Red ? offsetB.get() : -offsetB.get();

        Pose2d newPose = new Pose2d(
          originPose.getX() + xOffset, 
          MathUtil.clamp(robotPose.getY(),originPose.getY()-offsetB.get(),originPose.getY()+offsetB.get()), 
          originPose.getRotation()
        );

        Logger.recordOutput("BargeAlignCommand/targetPose", newPose);
        return newPose;
  }

  //Must be killed manually
  @Override
  public boolean isFinished() {
    return false;
  }
}

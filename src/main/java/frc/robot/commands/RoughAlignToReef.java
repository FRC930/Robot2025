package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import frc.robot.subsystems.vision.VisionConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;

public class RoughAlignToReef extends Command {
    private static int[] targetIdsRed = {
        6,7,8,9,10,11
    };

    private static int[] targetIdsBlue = {
        17,18,19,20,21,22
    };

    private static int[] targetIds;

    private LoggedTunableNumber offsetB = new LoggedTunableNumber("AutoAlign/offsetB", 0.4);
    private LoggedTunableNumber offsetR = new LoggedTunableNumber("AutoAlign/offsetR", 0);
    private boolean leftSide = false;

    private Drive drivetrain;
    private AprilTagFieldLayout fieldTags;
    private Trajectory trajectory;

    private Pose2d drivingPose = Pose2d.kZero;

    //The lerp. 0 at start, 1 at end
    private double i;

    /**
     * Returns the pose of the closest april tag in "targets" to "pos"
     * 
     * @param pos
     * @param targets
     * @return
     */
    private static Pose2d findClosestPose(Pose2d pos, int[] targets) {
        double minDistance = Double.MAX_VALUE;
        Pose2d target = Pose2d.kZero;
        
        for (int i = 0; i < targets.length; i++) {
            double distance = pos.getTranslation().getDistance(aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).getTranslation().toTranslation2d());
            if (distance < minDistance) {
                target = aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).toPose2d();
                minDistance = distance;
            }
        }
        
        return target;
    }

    public RoughAlignToReef(Drive drivetrain, boolean leftSide) {
        targetIds = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? targetIdsRed : targetIdsBlue ;
        this.drivetrain = drivetrain;
        this.fieldTags = VisionConstants.aprilTagLayout;
        this.leftSide = leftSide;
    }

    private void resetTargetPose() {
        double localoffsetR = leftSide ? offsetR.getAsDouble() * -1 : offsetR.getAsDouble();
        Transform2d offset = new Transform2d(offsetB.getAsDouble(), localoffsetR, Rotation2d.kZero);
        Pose2d closestPose = findClosestPose(drivetrain.getPose(), targetIds);

        Pose2d targetPose = closestPose.transformBy(offset);
        drivingPose = targetPose;
        Logger.recordOutput("ClosestPose",targetPose);
    }

    @Override
    public void initialize() {
        resetTargetPose();
        ArrayList<Pose2d> pointlist = new ArrayList<>();
        pointlist.add(drivetrain.getPose());
        pointlist.add(drivingPose.transformBy(new Transform2d(0.3, 0, Rotation2d.kZero)));
        pointlist.add(drivingPose);
        TrajectoryConfig configs = new TrajectoryConfig(2, 4);

        trajectory = TrajectoryGenerator.generateTrajectory(pointlist, configs);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;


public class FineAlignToReef extends Command {
    private PIDController skewController;
    private PIDController approachController;

    private static int[] targetIdsRed = {
        6,7,8,9,10,11
    };

    private static int[] targetIdsBlue = {
        17,18,19,20,21,22
    };

    private static int[] targetIds;

    public LoggedTunableNumber offsetB = new LoggedTunableNumber("AutoAlign/offsetB", 0.4);
    public LoggedTunableNumber offsetR = new LoggedTunableNumber("AutoAlign/offsetR", 0);
    
    private boolean leftSide = false;

    private Drive drivetrain;
    private int targetTag;

    /**
     * Returns the pose of the closest april tag in "targets" to "pos"
     * 
     * @param pos
     * @param targets
     * @return
     */
    private static int findClosestId(Pose2d pos, int[] targets) {
        double minDistance = Double.MAX_VALUE;
        int tID = -1;
        
        for (int i = 0; i < targets.length; i++) {
            double distance = pos.getTranslation().getDistance(aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).getTranslation().toTranslation2d());
            if (distance < minDistance) {
                tID = targets[i];
                minDistance = distance;
            }
        }
        
        return tID;
    }

    public FineAlignToReef(Drive drivetrain, boolean leftSide) {
        targetIds = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? targetIdsRed : targetIdsBlue ;
        this.drivetrain = drivetrain;
        this.leftSide = leftSide;
    }

    private void resetTargetId() {
        targetTag = findClosestId(, targetIds);
        //
    }

    @Override
    public void initialize() {
        resetTargetPose();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

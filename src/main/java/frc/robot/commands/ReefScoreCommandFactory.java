package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.Map;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;

public class ReefScoreCommandFactory {

    private static ReefScoreCommandFactory instance;

    public static enum ReefPosition {
        Left,
        Center,
        Right,
    }
    private static Alliance alliance;
    private final static int[] targetIdsRed = {
        6,7,8,9,10,11
    };

    private final static int[] targetIdsBlue = {
        17,18,19,20,21,22
    };

    private static int[] targetIds;

    //#region TODO Find Accurate Values
    private static LoggedTunableNumber offsetBBackingUp = new LoggedTunableNumber("AutoAlign/offsetBBackingUp", 2);
    private static LoggedTunableNumber rightOffsetBFinal = new LoggedTunableNumber("AutoAlign/rightOffsetBFinal", 0.55);
    private static LoggedTunableNumber leftOffsetBFinal = new LoggedTunableNumber("AutoAlign/leftOffsetBFinal", 0.45);
    private static LoggedTunableNumber offsetL = new LoggedTunableNumber("AutoAlign/offsetL", 0.3); //(papa smurf)
    private static LoggedTunableNumber offsetR = new LoggedTunableNumber("AutoAlign/offsetR", 0.14);
    //#endregion

    private static Drive drivetrain;

    /**
     * Finds the closest april tag to a position.
     * 
     * @param pos The Pose2d to find the closest relative tag.
     * @param targets The list of AprilTag IDs to check for.
     * @return The pose of the closest april tag in "targets" to "pos"
     */
    private static Pose2d findClosestPose(Pose2d pos) {
        int[] targets = targetIds;
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

    public static Function<Pose2d, Pose2d> getGetTargetPositionFunction(ReefPosition pos, boolean isBackingUp) {
        refreshAlliance();
        return (Pose2d pose) -> {
            double backOffset = leftOffsetBFinal.get();
            double appliedOffset = 0;
            switch (pos) {
                case Right:
                    appliedOffset = offsetR.getAsDouble();
                    backOffset = rightOffsetBFinal.get();
                    break;
                case Left:
                    appliedOffset = -offsetL.getAsDouble();
                    backOffset = leftOffsetBFinal.get();
                    break;
                case Center:
                default:
                    appliedOffset = 0;
                    break;
            }
            Transform2d offset = new Transform2d(isBackingUp ? offsetBBackingUp.getAsDouble() : backOffset, appliedOffset, Rotation2d.kZero);
            Pose2d closestTarget = findClosestPose(pose);

            Pose2d target = closestTarget.transformBy(offset);
            Logger.recordOutput("TargetPose",target);
            return target;
        };
    }

    public static void initialize(Drive d) {
        refreshAlliance();
        setDrivetrain(d);
    }

    public static void refreshAlliance() {
        targetIds = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? targetIdsBlue : targetIdsRed;
    }

    public static void setDrivetrain(Drive d) {
        drivetrain = d;
    }

    public static Command getNewAlignToReefCommand(ReefPosition position, boolean isBackingUp) {
        return new AutoAlignCommand(getGetTargetPositionFunction(position, isBackingUp), drivetrain);
    }

    public static Command getNewReefCoralScoreSequence(ReefPosition position, Map<ReefPositionsUtil.ScoreLevel,Command> coralLevelCommands, Map<ReefPositionsUtil.ScoreLevel,Command> scoreCoralLevelCommands) {
        return getNewAlignToReefCommand(position, true)
            .andThen(ReefPositionsUtil.getInstance().getCoralLevelSelector(coralLevelCommands))
            .andThen(getNewAlignToReefCommand(position, false))
            .andThen(ReefPositionsUtil.getInstance().getCoralLevelSelector(scoreCoralLevelCommands));
    }
}

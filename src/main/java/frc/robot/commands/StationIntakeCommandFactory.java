package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraLeft;

import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StationIntakeCommandFactory {

    public static enum IntakePosition {
        Inside,
        Center,
        Outside,
    }
    private final static int[] targetIdsRed = {
        1,2
    };

    private final static int[] targetIdsBlue = {
        12,13
    };

    private static int[] targetIds;
    private int selectedTargetId;

    //#region TODO Find Accurate Values
    private static LoggedTunableNumber offsetBBackingUp = new LoggedTunableNumber("StationAutoAlign/offsetBBackingUp", 0.75);

    // for tags 1 and 13

    private static LoggedTunableNumber outsideRightOffsetBFinal = new LoggedTunableNumber("StationAutoAlign/outsideRightOffsetBFinal", 0.4);
    private static LoggedTunableNumber outsideRightOffset = new LoggedTunableNumber("StationAutoAlign/outsideRightOffset", 0.5);

    private static LoggedTunableNumber insideLeftOffsetBFinal = new LoggedTunableNumber("StationAutoAlign/insideLeftOffsetBFinal", 0.4);
    private static LoggedTunableNumber insideLeftOffset = new LoggedTunableNumber("StationAutoAlign/insideLeftOffset", 0.5);

    // for tags 2 and 12

    private static LoggedTunableNumber insideRightOffsetBFinal = new LoggedTunableNumber("StationAutoAlign/insideRightOffsetBFinal", 0.4);
    private static LoggedTunableNumber insideRightOffset = new LoggedTunableNumber("StationAutoAlign/insideRightOffset", 0.5);

    private static LoggedTunableNumber outsideLeftOffsetBFinal = new LoggedTunableNumber("StationAutoAlign/outsideLeftOffsetBFinal", 0.4);
    private static LoggedTunableNumber outsideLeftOffset = new LoggedTunableNumber("StationAutoAlign/outsideLeftOffset", 0.5);
    //#endregion

    /**
     * Finds the closest april tag to a position.
     * 
     * @param pos The Pose2d to find the closest relative tag.
     * @param targets The list of AprilTag IDs to check for.
     * @return The pose of the closest april tag in "targets" to "pos"
     */
    private static int findClosestTargetId(Pose2d pos) {
        int[] targets = targetIds;
        double minDistance = Double.MAX_VALUE;
        int target = -1;
        
        for (int i = 0; i < targets.length; i++) {
            double distance = pos.getTranslation().getDistance(aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).getTranslation().toTranslation2d());
            if (distance < minDistance) {
                target = targets[i];
                minDistance = distance;
            }
        }
        
        return target;
    }

    /**
     * Returns A function which takes the current position on the robot and returns where we want to score.
     * @param pos
     * @param isBackingUp
     * @return
     */
    public static Function<Pose2d, Pose2d> getGetTargetPositionFunction(Supplier<IntakePosition> pos, boolean isBackingUp) {
        refreshAlliance();

        return (Pose2d pose) -> {
            double backOffset = insideLeftOffsetBFinal.get();
            double appliedOffset = 0;
            int closestTargetId = findClosestTargetId(pose);
            IntakePosition localIntakePos;
            if (pos == null || pos.get() == null) {
                localIntakePos = IntakePosition.Center;
                Logger.recordMetadata("StationIntakeCommandFactoryPos", "The Intake Position is at the Center.");
            } else {
                localIntakePos = pos.get();
                Logger.recordMetadata("StationIntakeCommandFactoryPos", "The Intake Position is at the position of Intake.");
            }

            switch (localIntakePos) {
                case Outside:
                    if (closestTargetId == 2 || closestTargetId == 12) {
                        appliedOffset = -outsideLeftOffset.getAsDouble();
                        backOffset = outsideLeftOffsetBFinal.get();
                        break;
                    } else {
                        appliedOffset = outsideRightOffset.getAsDouble();
                        backOffset = outsideRightOffsetBFinal.get();
                        break;
                    }
                case Inside:
                    if (closestTargetId == 2 || closestTargetId == 12) {
                        appliedOffset = insideRightOffset.getAsDouble();
                        backOffset = insideRightOffsetBFinal.get();
                        break;
                    } else {
                        appliedOffset = -insideLeftOffset.getAsDouble();
                        backOffset = insideLeftOffsetBFinal.get();
                        break;
                    }
                case Center:
                default:
                    appliedOffset = 0;
                    break;
            }
            Transform2d offset = new Transform2d(isBackingUp ? offsetBBackingUp.getAsDouble() : backOffset, appliedOffset, Rotation2d.k180deg);

            return aprilTagLayout.getTagPose(closestTargetId).orElse(Pose3d.kZero).toPose2d().transformBy(offset);
        };
    }

    public static void initialize() {
        refreshAlliance();
    }

    public static void refreshAlliance() {
        targetIds = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? targetIdsBlue : targetIdsRed;
    }

    /**
     * Gets a command which aligns the robot to the nearest reef position
     * @param position Left or right
     * @param isBackingUp Whether or not we should back up if we're too close to the reef
     */
    public static Command getNewAlignToStationCommand(Supplier<IntakePosition> position, boolean isBackingUp, Drive drive) {
        Function<Pose2d, Pose2d> positionFunction = getGetTargetPositionFunction(position, isBackingUp);
        //Base command
        Command returnedCommand = new AutoAlignCommand(getGetTargetPositionFunction(position, isBackingUp), drive);
        //If we're backing up, add a condition to kill when we're farther away than the backup distance
        if(isBackingUp) {
            returnedCommand = returnedCommand.until(() -> (drive.getDistanceTo(positionFunction.apply(drive.getPose())).in(Meters) > offsetBBackingUp.getAsDouble()));
        }
        return returnedCommand;
    }

    /**
     * Gets a command which aligns the robotto the nearest reef position and prepares to score the coal, backing up if it's too close
     * @param position
     * @param coralLevelCommands
     * @param scoreCoralLevelCommands
     * @param drive
     * @return
     */
    public static Command getNewStationIntakeSequence(Supplier<IntakePosition> position, ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, Drive drive) {
        return new StationIntakeCommand(shoulder, elbow, elevator, wrist, coralEE)
            .andThen(
                getNewAlignToStationCommand(position, false, drive)
            );
    }
}

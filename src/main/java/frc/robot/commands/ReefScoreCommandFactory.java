package frc.robot.commands;

import java.util.Map;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;
import frc.robot.util.SelectorCommandFactory;
import frc.robot.util.ReefPositionsUtil.DeAlgaeLevel;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

public class ReefScoreCommandFactory {

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

    private final static int[] targetIdsAll = {
        6,7,8,9,10,11,
        17,18,19,20,21,22
    };

    private static int[] targetIds;

    //#region Coral Alignment
    private static LoggedTunableNumber offsetBBackingUp = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Back/BackingUp", 1.0);
    private static LoggedTunableNumber offsetBDefault = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Back/default", 0.68);

    private static LoggedTunableNumber defaultOffsetL = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Left/default", 0.155 + Inches.of(1).in(Meters));
    private static LoggedTunableNumber defaultOffsetR = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Right/default", 0.2 - Inches.of(3).in(Meters));
    //#endregion

    //#region Algae Alignment
    private static LoggedTunableNumber algaeOffsetBFinal = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Algae/BackOffset", 0.53);
    private static LoggedTunableNumber offsetCR = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Algae/RightOffset", Inches.of(0).in(Meters));
    //#endregion

    //Overrides
        //Left level overrides
    private static LoggedTunableNumber offsetLL1 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Left/L1", 0.1804);
    private static LoggedTunableNumber offsetLL2 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Left/L2", 0.18);
    private static LoggedTunableNumber offsetLL3 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Left/L3", 0.19);
    private static LoggedTunableNumber offsetLL4 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Left/L4", 0.22);
        //Right level overrides
    private static LoggedTunableNumber offsetRL1 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Right/L1", 0.1238);
    private static LoggedTunableNumber offsetRL2 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Right/L2", 0.15);
    private static LoggedTunableNumber offsetRL3 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Right/L3", 0.16);
    private static LoggedTunableNumber offsetRL4 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Right/L4", 0.09);
        //Back level overrides (Overrides on how far back each level will be scored)
    private static LoggedTunableNumber offsetBFinalL1 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Back/L1", 0.638);
    private static LoggedTunableNumber offsetBFinalL2 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Back/L2", 0.7);
    private static LoggedTunableNumber offsetBFinalL3 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Back/L3", 0.71);
    private static LoggedTunableNumber offsetBFinalL4 = new LoggedTunableNumber("AutoAlignCommands/ReefAlignCommand/Offsets/Back/L4", 0.685);

    /**
     * Finds the closest reef april tag (from the list of this alliance's apriltags) to a position.
     * 
     * @param pos The Pose2d to find the closest relative tag.
     * @param targets The list of AprilTag IDs to check for.
     * @return The pose of the closest april tag in "targets" to "pos"
     */
    private static Pose2d findClosestTag(Pose2d pos) {
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

    private static Pose2d findClosestTagFromBothAlliances(Pose2d pos) {
        int[] targets = targetIdsAll;
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

    /**
     * Returns A function which takes the current position on the robot and returns where we want to score.
     * @param pos
     * @param isBackingUp
     * @return
     */
    public static Function<Pose2d, Pose2d> getGetTargetPositionFunction(ReefPosition pos, boolean isBackingUp) {
        refreshAlliance();
        return (Pose2d pose) -> {
            // Set default offsets
            double backOffset = offsetBDefault.getAsDouble();
            double sideOffset = 0;
            // Select between different alignments for different levels
            double offsetRForLevel = defaultOffsetR.getAsDouble();
            double offsetLForLevel = defaultOffsetL.getAsDouble();
            double offsetCForLevel = offsetCR.getAsDouble();

            switch(ReefPositionsUtil.getInstance().getScoreLevel()) {
                case L1:
                    offsetRForLevel = offsetRL1.getAsDouble();
                    offsetLForLevel = offsetLL1.getAsDouble();
                    backOffset = offsetBFinalL1.getAsDouble();
                    break;
                case L2:
                    offsetRForLevel = offsetRL2.getAsDouble();
                    offsetLForLevel = offsetLL2.getAsDouble();
                    backOffset = offsetBFinalL2.getAsDouble();
                    break;
                case L3:
                    offsetRForLevel = offsetRL3.getAsDouble();
                    offsetLForLevel = offsetLL3.getAsDouble();
                    backOffset = offsetBFinalL3.getAsDouble();
                    break;
                case L4:
                    offsetRForLevel = offsetRL4.getAsDouble();
                    offsetLForLevel = offsetLL4.getAsDouble();
                    backOffset = offsetBFinalL4.getAsDouble();
                    break;
                default:
                    offsetRForLevel = defaultOffsetR.getAsDouble();
                    offsetLForLevel = defaultOffsetL.getAsDouble();
                    backOffset      = offsetBDefault.getAsDouble();
                    break;
            }
            // Select between different alignments for different sides
            switch (pos) {
                case Right:
                    sideOffset = offsetRForLevel;
                    break;
                case Left:
                    sideOffset = -offsetLForLevel;
                    break;
                case Center:
                    sideOffset = offsetCR.getAsDouble();
                default:
                    sideOffset = -offsetCForLevel;
                    backOffset = algaeOffsetBFinal.get();
                    break;
            }

            // If we're backing up override our back offset
            if(isBackingUp) {
                backOffset = offsetBBackingUp.getAsDouble();
            }
            
            //Get our closest tag
            Pose2d closestTarget = pos == ReefPosition.Center ? findClosestTagFromBothAlliances(pose) : findClosestTag(pose);
            //Build a transformation for our offset from that tag
            Transform2d offset = new Transform2d(backOffset, sideOffset, Rotation2d.kZero);

            //Transform our tag to get our final target pose and log it
            Pose2d target = closestTarget.transformBy(offset);
            Logger.recordOutput("TargetPose",target);

            //Return the target pose
            return target;
        };
    }

    /**
     * Sets our alliance to the one returned by getalliance, defaulting to red.
     */
    public static void refreshAlliance() {
        alliance = DriverStation.getAlliance().orElse(null);
        if(alliance == null) {
            alliance = Alliance.Red;
            //TODO: Add safety kill for if we didn't get an alliance from the DS
        }
        targetIds = alliance == Alliance.Blue ? targetIdsBlue : targetIdsRed;
    }

    public static void initialize() {
        //Refresh our alliance every time we init this command
        refreshAlliance();
    }

    /**
     * Gets a command which aligns the robot to the nearest reef position
     * @param position Left or right
     * @param isBackingUp Whether or not we should back up if we're too close to the reef
     */
    public static Command getNewAlignToReefCommand(ReefPosition position, boolean isBackingUp, Drive drive) {
        Function<Pose2d, Pose2d> positionFunction = getGetTargetPositionFunction(position, isBackingUp);
        //Base command
        Command returnedCommand = new AutoAlignCommand(getGetTargetPositionFunction(position, isBackingUp), drive);
        //If we're backing up, add kill conditions
        if(isBackingUp) {
            // returnedCommand = returnedCommand
                // Kill when we are out of the distance (not necessary since we kill)
                // .until(() -> (drive.getDistanceTo(positionFunction.apply(drive.getPose())).in(Meters) > offsetBBackingUp.getAsDouble()))
                // Don't run the backup if we are out of the distance
                // .unless(() -> {
                //     double dist = drive.getDistanceTo(positionFunction.apply(drive.getAutoAlignPose())).in(Meters);
                //     Logger.recordOutput("AutoAlign/DistanceMeters",dist);
                //     return dist > offsetBBackingUp.getAsDouble();
                // });
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
    public static Command getNewReefCoralScoreSequence(ReefPosition position, boolean isBackingUp, Map<ReefPositionsUtil.ScoreLevel,Command> coralLevelCommands, Map<ReefPositionsUtil.ScoreLevel,Command> scoreCoralLevelCommands, Map<ReefPositionsUtil.ScoreLevel,Command> stopCoralLevelCommands, Map<ReefPositionsUtil.ScoreLevel,Command> waitUntilCoralLevelCommands, Drive drive) {
        ReefPositionsUtil reefUtil = ReefPositionsUtil.getInstance();
        return 
            getNewAlignToReefCommand(position, true, drive).onlyIf(()->isBackingUp)
                .andThen(DriveCommands.brakeDrive(drive))
                .withDeadline(
                    reefUtil.getCoralLevelSelector(coralLevelCommands)
                    .andThen(reefUtil.getCoralLevelSelector(waitUntilCoralLevelCommands))
                )
            .andThen(getNewAlignToReefCommand(position, false, drive))
            .andThen(reefUtil.getCoralLevelSelector(scoreCoralLevelCommands))
            // Added wait for score L4 so no need for wait here
            .andThen(
                getNewAlignToReefCommand(position, true, drive).onlyIf(()->reefUtil.isSelected(ScoreLevel.L4))
                    .andThen(reefUtil.getCoralLevelSelector(stopCoralLevelCommands))
            );
    }

    public static Command getNewReefCoralScoreSequence(ReefPosition position, Boolean isBackingUp, Drive drive, ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE) {
        return getNewReefCoralScoreSequence(position, false, SelectorCommandFactory.getCoralLevelPrepCommandSelector(shoulder, elbow, elevator, wrist), SelectorCommandFactory.getCoralLevelScoreCommandSelector(shoulder, elbow, elevator, wrist, coralEE), SelectorCommandFactory.getCoralLevelStopScoreCommandSelector(elbow, wrist, coralEE, drive), SelectorCommandFactory.getCoralLevelWaitUntilAtLevelCommandSelector(shoulder, elbow, elevator, wrist), drive);
    }

    public static Command getNewReefCoralScoreSequence(ReefPosition position, boolean isBackingUp, Drive drive) {
        return getNewAlignToReefCommand(position, true, drive).onlyIf(()->isBackingUp)
            .andThen(DriveCommands.brakeDrive(drive))
            .andThen(getNewAlignToReefCommand(position, false, drive));
    }

    /**
     * Gets a command that scores an algae in auto
     * @param drive
     * @param shoulder
     * @param elbow
     * @param elevator
     * @param wrist
     * @param coralEE
     * @param algaeEE
     * @return
     */
    public static Command getNewAlgaePluckAutoAlignSequenceCommand(
        DeAlgaeLevel level, 
        Drive drive,
        ArmJoint shoulder, 
        ArmJoint elbow, 
        Elevator elevator, 
        Wrist wrist, 
        AlgaeEndEffector algaeEE
    ) {
        return getNewAlignToReefCommand(ReefPosition.Center, true, drive)
            .andThen(new ConditionalCommand(
                new TakeAlgaeL2(shoulder, elbow, wrist, algaeEE, elevator),
                new TakeAlgaeL3(shoulder, elbow, wrist, algaeEE, elevator),
                () -> ReefPositionsUtil.getInstance().isSelected(DeAlgaeLevel.Low)))
            .andThen(getNewAlignToReefCommand(ReefPosition.Center, false, drive))
            .until(algaeEE.hasAlgaeTrigger().debounce(0.5))
            .andThen(new WaitUntilCommand(algaeEE.hasAlgaeTrigger()))
            .andThen(getNewAlignToReefCommand(ReefPosition.Center, true, drive))
            .andThen(new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEE))
            .andThen(new InstantCommand(()-> {
                if(ReefPositionsUtil.getInstance().isSelected(DeAlgaeLevel.Low)){
                System.out.println("LOWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");
                }
                else{
                    System.out.println("HIGHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
                }
            }));
    }
    
    public static Command getNewAlgaePluckAutoAlignCommand(
        Drive drive,
        boolean isBackingUp
    ) {
        return getNewAlignToReefCommand(ReefPosition.Center, isBackingUp, drive);
    }
}

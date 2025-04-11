package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AlgaeStowCommand;
import frc.robot.commands.BargeScoreThrowCommand;
import frc.robot.commands.L4ToStow;
import frc.robot.commands.OutakeAlgae;
import frc.robot.commands.OutakeCoral;
import frc.robot.commands.ReefScoreCommandFactory;
import frc.robot.commands.ReefScoreCommandFactory.ReefPosition;
import frc.robot.commands.StationIntakeCommand;
import frc.robot.commands.StationIntakeCommandFactory;
import frc.robot.commands.StationIntakeCommandFactory.IntakePosition;
import frc.robot.commands.StationIntakeReverseCommand;
import frc.robot.commands.StationIntakeToStow;
import frc.robot.commands.StopDrivetrainCommand;
import frc.robot.commands.StowCommand;
import frc.robot.commands.StowToBarge;
import frc.robot.commands.StowToL1;
import frc.robot.commands.StowToL2;
import frc.robot.commands.StowToL3;
import frc.robot.commands.StowToL4;
import frc.robot.commands.TakeAlgaeL2;
import frc.robot.commands.TakeAlgaeL3;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.ReefPositionsUtil;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;


public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(Drive drive, ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
    configureNamedCommands(drive, shoulder, elbow, elevator, wrist, coralEE, algaeEE);
    

    // PathPlannerAuto CharacterizationTest = new PathPlannerAuto("CharacterizeAuto");
    // PathPlannerAuto ChoreoStraightAuto = new PathPlannerAuto("ChoreoStraightAuto");

     // Set up auto routines
     autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));


    // // Add Choreo Paths
    // autoChooser.addOption("Characterization Test Path", CharacterizationTest);
    // autoChooser.addOption("[Choreo] Straight Test", ChoreoStraightAuto);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureNamedCommands(Drive drive, ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
    //#region Stows
    NamedCommands.registerCommand("Stow", new StowCommand(shoulder, elbow, elevator, wrist, coralEE, algaeEE));
    NamedCommands.registerCommand("UnsafeStow", StowCommand.getNewUnsafeStowCommand(shoulder, elbow, elevator, wrist, coralEE, algaeEE));
    NamedCommands.registerCommand("ElevatorStow", L4ToStow.getNewElevatorToStowCommand(elevator));
    NamedCommands.registerCommand("L4ToStow", new L4ToStow(shoulder, elbow, elevator, wrist, coralEE, algaeEE));
    NamedCommands.registerCommand("AlgaeStow", new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEE));
    // Needed so not hit coral on elevator
    NamedCommands.registerCommand("StationIntakeToStow", new StationIntakeToStow(shoulder, elbow, elevator, wrist, coralEE, algaeEE));
    NamedCommands.registerCommand("UnsafeStationIntakeToStow", StationIntakeToStow.getNewUnsafeStationIntakeToStow(shoulder, elbow, elevator, wrist, coralEE, algaeEE));
    NamedCommands.registerCommand("WaitUntilElevatorStow", new WaitUntilCommand(elevator.getNewAtDistanceTrigger(Inches.of(0.0), Inches.of(1.0))));
    //#endregion

    //#region Intakes
    NamedCommands.registerCommand("StationIntake", new StationIntakeCommand(shoulder, elbow, elevator, wrist, coralEE));
    NamedCommands.registerCommand("ReverseStationIntake", new StationIntakeReverseCommand(shoulder, elbow, elevator, wrist, coralEE));
    NamedCommands.registerCommand("TakeAlgaeL2",
      new TakeAlgaeL2(shoulder, elbow, wrist, algaeEE, elevator)
    );
    NamedCommands.registerCommand("TakeAlgaeL3",
      new TakeAlgaeL3(shoulder, elbow, wrist, algaeEE, elevator)
    );
    //#endregion

    //#region Prep Scores
    NamedCommands.registerCommand("StowToL1", new StowToL1(shoulder, elbow, wrist));
    NamedCommands.registerCommand("StowToL2", new StowToL2(shoulder, elbow, elevator, wrist));
    NamedCommands.registerCommand("StowToL3", new StowToL3(shoulder, elbow, wrist, elevator));
    NamedCommands.registerCommand("StowToL4", new StowToL4(shoulder, elbow, elevator, wrist));
    NamedCommands.registerCommand("StowToL4Elevator", new StowToL4(elevator, shoulder));
    NamedCommands.registerCommand("StowToL4Arm", new StowToL4(shoulder, elbow, wrist));
    NamedCommands.registerCommand("WaitUntilL4", 
      new StopDrivetrainCommand(drive)
        .unless(StowToL4.getNewAtScoreTrigger(shoulder, elbow, elevator, wrist))
        .andThen(new WaitUntilCommand(StowToL4.getNewAtScoreTrigger(shoulder, elbow, elevator, wrist)).withTimeout(1.5)));

      NamedCommands.registerCommand("WaitUntilArmAtL4", 
      new StopDrivetrainCommand(drive)
        .unless(StowToL4.getNewArmAtL4Trigger(shoulder, elbow, wrist))
        .andThen(new WaitUntilCommand(StowToL4.getNewArmAtL4Trigger(shoulder, elbow, wrist))));
    //#endregion

    //#region Confirm Scores
    NamedCommands.registerCommand("ScoreL1", StowToL1.getNewScoreCommand(coralEE));
    NamedCommands.registerCommand("ScoreL2", StowToL2.getNewScoreCommand(shoulder, elbow, wrist, coralEE));
    NamedCommands.registerCommand("ScoreL3", StowToL3.getNewScoreCommand(shoulder, elbow, wrist, coralEE));
    NamedCommands.registerCommand("ScoreL4", StowToL4.getNewScoreCommand(elbow, wrist, coralEE));
    //#endregion

    //#region StopScore/Backup
    NamedCommands.registerCommand("StopScoreL1", StowToL1.getNewStopScoreCommand(coralEE));
    NamedCommands.registerCommand("StopScoreL2", StowToL2.getNewStopScoreCommand(elbow, wrist, coralEE));
    NamedCommands.registerCommand("StopScoreL3", StowToL3.getNewStopScoreCommand(elbow, wrist, coralEE));
    NamedCommands.registerCommand("StopScoreL4", StowToL4.getNewStopScoreCommand(elbow, wrist, coralEE, drive));
    NamedCommands.registerCommand("BackupL4Left", 
      ReefScoreCommandFactory.getNewAlignToReefCommand(
        ReefPosition.Left, 
        true, 
        drive
    ));
    NamedCommands.registerCommand("BackupL4Right", 
      ReefScoreCommandFactory.getNewAlignToReefCommand(
        ReefPosition.Right, 
        true, 
        drive
    ));
    //#endregion

    NamedCommands.registerCommand("OutakeCoral", new OutakeCoral(coralEE));
    NamedCommands.registerCommand("OutakeAlgae", new OutakeAlgae(algaeEE));
    NamedCommands.registerCommand("StopDrivetrain", new StopDrivetrainCommand(drive));
    NamedCommands.registerCommand("StopUnlessL4", 
      new StopDrivetrainCommand(drive)
        .unless(StowToL4.getNewAtScoreTrigger(shoulder, elbow, elevator, wrist))
    );

    // Deprecated
    NamedCommands.registerCommand("SetL1", ReefPositionsUtil.getInstance().getNewSetScoreLevelCommand(ScoreLevel.L1));
    NamedCommands.registerCommand("SetL2", ReefPositionsUtil.getInstance().getNewSetScoreLevelCommand(ScoreLevel.L2));
    NamedCommands.registerCommand("SetL3", ReefPositionsUtil.getInstance().getNewSetScoreLevelCommand(ScoreLevel.L3));
    NamedCommands.registerCommand("SetL4", ReefPositionsUtil.getInstance().getNewSetScoreLevelCommand(ScoreLevel.L4));

    //#region Auto Align
    NamedCommands.registerCommand("AutoAlignStationInside", 
      StationIntakeCommandFactory.getNewAlignToStationCommand(
        () -> {return IntakePosition.Inside;}, 
        false, 
        drive
      ).andThen(new WaitUntilCommand(coralEE.hasCoralTrigger()).withTimeout(2.0)).until(coralEE.hasCoralTrigger())
    );
    NamedCommands.registerCommand("AutoAlignStationCenter", 
      StationIntakeCommandFactory.getNewAlignToStationCommand(
        () -> {return IntakePosition.Center;}, 
        false,
        drive
      ).andThen(new WaitUntilCommand(coralEE.hasCoralTrigger()).withTimeout(2.0)).until(coralEE.hasCoralTrigger())
    );
    NamedCommands.registerCommand("AutoAlignStationOutside", 
      StationIntakeCommandFactory.getNewAlignToStationCommand(
        () -> {return IntakePosition.Outside;},
        false,
        drive
      ).andThen(new WaitUntilCommand(coralEE.hasCoralTrigger()).withTimeout(2.0)).until(coralEE.hasCoralTrigger())
    );

    NamedCommands.registerCommand("AutoAlignScoreLeft", 
      ReefScoreCommandFactory.getNewReefCoralScoreSequence(
          ReefPosition.Left, 
          false,
          drive));
    NamedCommands.registerCommand("AutoAlignScoreRight", 
      ReefScoreCommandFactory.getNewReefCoralScoreSequence(
        ReefPosition.Right, 
        false,
        drive));
    
    NamedCommands.registerCommand("AutoAlignAlgaePluck",
      ReefScoreCommandFactory.getNewAlgaePluckAutoAlignCommand(drive, false)
    );

    NamedCommands.registerCommand("AutoAlignAlgaePluckBackup",
      ReefScoreCommandFactory.getNewAlgaePluckAutoAlignCommand(drive, true)
    );

    NamedCommands.registerCommand("BargeScore",
      new BargeScoreThrowCommand(elevator, wrist, algaeEE)
      .andThen(new WaitUntilCommand(algaeEE.hasAlgaeTrigger().negate()).withTimeout(3))
    );

    NamedCommands.registerCommand("StowToBarge",
      new StowToBarge(shoulder, elbow, wrist)
    );

    NamedCommands.registerCommand("BargeStow",
      new ConditionalCommand(
        new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEE),
        elevator.getNewSetDistanceCommand(0.0)
        .andThen(new WaitUntilCommand(elevator.getNewLessThanDistanceTrigger(() -> 5.0))).withTimeout(0.5)
        .andThen(new StowCommand(shoulder, elbow, elevator, wrist, coralEE, algaeEE)),
        algaeEE.hasAlgaeTrigger()
      )
    );
    //#endregion

    //#region Full Sequences
    NamedCommands.registerCommand("FullAutoAlignSequenceLeft", 
      ReefScoreCommandFactory.getNewReefCoralScoreSequence(
        ReefPosition.Left, 
        true, 
        drive, shoulder, elbow, elevator, wrist, coralEE
      )
    );

    NamedCommands.registerCommand("FullAutoAlignSequenceRight", 
      ReefScoreCommandFactory.getNewReefCoralScoreSequence(
        ReefPosition.Right, 
        true, 
        drive, shoulder, elbow, elevator, wrist, coralEE
      )
    );
    //#endregion
  }
}

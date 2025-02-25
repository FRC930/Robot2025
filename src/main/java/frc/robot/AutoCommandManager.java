package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.OutakeCoral;
import frc.robot.commands.StopDrivetrainCommand;
import frc.robot.commands.StowCommand;
import frc.robot.commands.StowToL1;
import frc.robot.commands.StowToL2;
import frc.robot.commands.StowToL3;
import frc.robot.commands.StowToL4;
import frc.robot.commands.StationIntakeReverseCommand;
import frc.robot.commands.StationIntakeCommand;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(Drive drive, ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
    configureNamedCommands(drive, shoulder, elbow, elevator, wrist, coralEE, algaeEE);;
    

    PathPlannerAuto CharacterizationTest = new PathPlannerAuto("CharacterizeAuto");
    PathPlannerAuto ChoreoStraightAuto = new PathPlannerAuto("ChoreoStraightAuto");

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));


    // Add Choreo Paths
    autoChooser.addOption("Characterization Test Path", CharacterizationTest);
    autoChooser.addOption("[Choreo] Straight Test", ChoreoStraightAuto);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureNamedCommands(Drive drive, ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
      NamedCommands.registerCommand("Stow", new StowCommand(shoulder, elbow, elevator, wrist, coralEE, algaeEE));
      NamedCommands.registerCommand("StationIntake", new StationIntakeCommand(shoulder, elbow, elevator, wrist, coralEE));
      NamedCommands.registerCommand("StartIntake", new StationIntakeReverseCommand(shoulder, elbow, elevator, wrist, coralEE));
      NamedCommands.registerCommand("StowToL1", new StowToL1(shoulder, elbow, wrist));
      NamedCommands.registerCommand("StowToL2", new StowToL2(shoulder, elbow, elevator, wrist));
      NamedCommands.registerCommand("StowToL3", new StowToL3(shoulder, elbow, wrist, elevator));
      NamedCommands.registerCommand("StowToL4", new StowToL4(shoulder, elbow, elevator, wrist));
      NamedCommands.registerCommand("ScoreL1",StowToL1.getNewScoreCommand(coralEE));
      NamedCommands.registerCommand("ScoreL2",StowToL2.getNewScoreCommand(elbow, wrist, coralEE));
      NamedCommands.registerCommand("ScoreL3",StowToL3.getNewScoreCommand(elbow, wrist, coralEE));
      NamedCommands.registerCommand("ScoreL4",StowToL4.getNewScoreCommand(elbow, wrist, coralEE));
      NamedCommands.registerCommand("StopScoreL1",StowToL1.getNewStopScoreCommand(coralEE));
      NamedCommands.registerCommand("StopScoreL2",StowToL2.getNewStopScoreCommand(elbow, wrist, coralEE));
      NamedCommands.registerCommand("StopScoreL3",StowToL3.getNewStopScoreCommand(elbow, wrist, coralEE));
      NamedCommands.registerCommand("StopScoreL4",StowToL4.getNewStopScoreCommand(elbow, wrist, coralEE));
  
      NamedCommands.registerCommand("CoralOuttake", new OutakeCoral(coralEE));
      NamedCommands.registerCommand("StopDrivetrain", new StopDrivetrainCommand(drive));
    
  }
}

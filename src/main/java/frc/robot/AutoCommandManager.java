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
import frc.robot.commands.StowToL4;
import frc.robot.commands.TakeCoral;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class AutoCommandManager {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoCommandManager(Drive drive, ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
    configureNamedCommands(drive, shoulder, elbow, elevator, wrist, coralEE, algaeEE);;
    

    PathPlannerAuto CharacterizationTest = new PathPlannerAuto("CharacterizeAuto");
    PathPlannerAuto ChoreoStraightAuto = new PathPlannerAuto("ChoreoStraightAuto");

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    PathPlannerAuto Circle = new PathPlannerAuto("Circle");
    PathPlannerAuto Straight = new PathPlannerAuto("Straight");
    PathPlannerAuto Diagonal = new PathPlannerAuto("Diagonal");
    PathPlannerAuto simpleAuto = new PathPlannerAuto("simpleAuto");
    PathPlannerAuto advancedPath = new PathPlannerAuto("advancedPath");
    PathPlannerAuto exitZone = new PathPlannerAuto("exitZone");

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
    autoChooser.addOption("Circle", Circle);
    autoChooser.addOption("Straight", Straight);
    autoChooser.addOption("Diagonal", Diagonal);
    autoChooser.addOption("simpleAuto", simpleAuto);
    autoChooser.addOption("advancedPath", advancedPath);
    autoChooser.addOption("exitZone", exitZone);


    // Add Choreo Paths
    autoChooser.addOption("Characterization Test Path", CharacterizationTest);
    autoChooser.addOption("[Choreo] Straight Test", ChoreoStraightAuto);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void configureNamedCommands(Drive drive, ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
      NamedCommands.registerCommand("StopIntake", new StowCommand(shoulder, elbow, elevator, wrist, coralEE, algaeEE));
      NamedCommands.registerCommand("StartIntake", new TakeCoral(shoulder, elbow, elevator, wrist, coralEE));
      NamedCommands.registerCommand("L4Score", new StowToL4(shoulder, elbow, elevator, wrist, coralEE));
      NamedCommands.registerCommand("L1Score", new StowToL1(shoulder, elbow, wrist, coralEE));
      NamedCommands.registerCommand("CoralOuttake", new OutakeCoral(coralEE));
      NamedCommands.registerCommand("L2Score", new PrintCommand("***************Scoring L2 sir!"));
      NamedCommands.registerCommand("StopDrivetrain", new StopDrivetrainCommand(drive));
  }
}

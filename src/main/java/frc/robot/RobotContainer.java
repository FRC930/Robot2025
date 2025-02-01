// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here. *
 */
package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.limelightBackName;
import static frc.robot.subsystems.vision.VisionConstants.limelightFrontName;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraBack;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraFront;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.arm.ArmJointIOSim;
import frc.robot.subsystems.arm.ArmJointIOTalonFX;
import frc.robot.subsystems.arm.constants.ElbowConstants;
import frc.robot.subsystems.arm.constants.ShoulderConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.fingeys.FingeysConstants;
import frc.robot.subsystems.fingeys.FingeysIOSim;
import frc.robot.subsystems.fingeys.FingeysIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.intakeextender.IntakeExtenderIOSim;
import frc.robot.subsystems.intakeextender.IntakeExtenderIOTalonFX;
import frc.robot.subsystems.toesies.Toesies;
import frc.robot.subsystems.toesies.ToesiesIOSim;
import frc.robot.subsystems.toesies.ToesiesIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.fingeys.FingeysIOSim;
import frc.robot.subsystems.fingeys.FingeysIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.LoggedTunableNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // TODO: SET PROPER MOTOR IDS
  private final double DRIVE_SPEED = 1.0;
  private final double ANGULAR_SPEED = 0.75;
  private final Wrist wrist;

  private final ArmJoint shoulder;
  private final ArmJoint elbow;

  private final Elevator elevator;

  private final Fingeys fingeys;

  private final Intake intake;

  private final Toesies toesies;

  private final IntakeExtender intakeExtender;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController co_controller = new CommandXboxController(1);
  private final CommandXboxController characterizeController = new CommandXboxController(2);
  private final CommandXboxController testcontroller = new CommandXboxController(3);

  private final AprilTagVision vision;

  private AutoCommandManager autoCommandManager;
  private RobotState robotState;

  private boolean m_TeleopInitialized = false;

  final LoggedTunableNumber setElevatorDistance = new LoggedTunableNumber("RobotState/Elevator/setDistance", 58);
  final LoggedTunableNumber setWristAngle = new LoggedTunableNumber("RobotState/Wrist/setAngle", 90);
  final LoggedTunableNumber setToesiesVolts = new LoggedTunableNumber("RobotState/Toesies/setVolts", 2);
  final LoggedTunableNumber setFingeysVolts = new LoggedTunableNumber("RobotState/Fingeys/setVolts", 2);
  final LoggedTunableNumber setIntakeExtenderAngle = new LoggedTunableNumber("RobotState/IntakeExtender/setAngle", 90);
  final LoggedTunableNumber setIntakeVolts = new LoggedTunableNumber("RobotState/Intake/setVolts", 2);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurement,
                new VisionIOLimelight(limelightFrontName, drive::getRotation),
                new VisionIOLimelight(limelightBackName, drive::getRotation));

        wrist = new Wrist(new WristIOTalonFX(11));

        elevator = new Elevator(new ElevatorIOTalonFX(13, 14));

        shoulder = new ArmJoint( new ArmJointIOTalonFX(new ShoulderConstants()));
        elbow = new ArmJoint( new ArmJointIOTalonFX(new ElbowConstants()));

        fingeys = new Fingeys( new FingeysIOTalonFX(12));
        
        intake = new Intake( new IntakeIOTalonFX(15, 17));

        intakeExtender = new IntakeExtender( new IntakeExtenderIOTalonFX(16));

        toesies = new Toesies( new ToesiesIOTalonFX(6));

        // arm = new ArmJoint(new ArmJointIOTalonFX(), null);

        // vision =
        //     new Vision(
        //         demoDrive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));
        // arm = new ArmJoint(new ArmJointIOTalonFX(), null);


        // Real robot, instantiate hardware IO implementations
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new AprilTagVision(
                drive::setPose,
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(limelightFrontName, robotToCameraFront, drive::getPose),
                new VisionIOPhotonVisionSim(limelightBackName, robotToCameraBack, drive::getPose));

        wrist = new Wrist(new WristIOSim(3));
        elevator = new Elevator(new ElevatorIOSim(4,new ElevatorSim(0.5, 0.2, DCMotor.getKrakenX60Foc(1), Meters.convertFrom(40.75, Inches), Meters.convertFrom(68.25, Inches), false, Meters.convertFrom(40.75, Inches), 0.001, 0.001)));

        shoulder = new ArmJoint(new ArmJointIOSim(new ShoulderConstants()));
        elbow = new ArmJoint(new ArmJointIOSim(new ElbowConstants()));
        fingeys = new Fingeys(new FingeysIOSim(121));
        intake = new Intake(new IntakeIOSim(15));
        toesies = new Toesies(new ToesiesIOSim(12));
        intakeExtender = new IntakeExtender( new IntakeExtenderIOSim(16));
        
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision =
            new AprilTagVision(
                drive::setPose, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        wrist = null;
        elevator = null;
        shoulder = null;
        elbow = null;
        fingeys = null;
        intake = null;
        toesies = null;
        intakeExtender = null;
        break;
    }

    autoCommandManager = new AutoCommandManager(drive);

    // Configure the button bindings
    configureButtonBindings();
    configureTestButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}. Used for getting SysIDs
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * DRIVE_SPEED,
            () -> -controller.getLeftX() * DRIVE_SPEED,
            () -> -controller.getRightX() * ANGULAR_SPEED)
          );

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    characterizeController
        .back()
        .and(characterizeController.y())
        .whileTrue(drive.sysIdDynamic(Direction.kForward));
    characterizeController
        .back()
        .and(characterizeController.x())
        .whileTrue(drive.sysIdDynamic(Direction.kReverse));
    characterizeController
        .start()
        .and(characterizeController.y())
        .whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    characterizeController
        .start()
        .and(characterizeController.x())
        .whileTrue(drive.sysIdQuasistatic(Direction.kReverse));
    characterizeController
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  SignalLogger.setPath("/media/sda1/logs");
                  // SignalLogger.enableAutoLogging(true);
                  SignalLogger.start();
                  System.out.println("Started Logger");
                }));
    characterizeController
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  SignalLogger.stop();
                  System.out.println("Stopped Logger");
                }));
    
    controller.rightBumper()
    .onTrue(
      elbow.getNewSetAngleCommand(-30).alongWith(shoulder.getNewSetAngleCommand(75))
      .andThen(new WaitUntilCommand(elbow.getNewAtSetpointTrigger().and(shoulder.getNewAtSetpointTrigger())))
      .andThen(
        elbow.getNewSetAngleCommand(70)
        .until(elbow.getNewAtSetpointTrigger().and(shoulder.getNewAtSetpointTrigger()))
      )
    
    ).onFalse(elbow.getNewSetAngleCommand(0).alongWith(shoulder.getNewSetAngleCommand(0)));

    // Auto aim command example FOR DIFFERENTIAL DRIVE
    // @SuppressWarnings("resource")
    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // keyboard
    //     .button(1)
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //               drive.run(0.0, aimController.calculate(vision.getTargetX(0).getRadians()));
    //             },
    //             drive));
  }

  public void configureTestButtonBindings (){
    testcontroller.y().onTrue(elevator.getNewSetDistanceCommand(setElevatorDistance)).onFalse(elevator.getNewSetDistanceCommand(0));
    testcontroller.leftBumper().onTrue(wrist.getNewWristTurnCommand(setWristAngle)).onFalse(wrist.getNewWristTurnCommand(0));
    testcontroller.rightBumper().onTrue(toesies.getNewSetVoltsCommand(setToesiesVolts)).onFalse(toesies.getNewSetVoltsCommand(0));
    testcontroller.leftTrigger().onTrue(fingeys.getNewSetVoltsCommand(setFingeysVolts)).onFalse(fingeys.getNewSetVoltsCommand(0));
    testcontroller.rightTrigger().onTrue(intake.getNewSetVoltsCommand(setIntakeVolts)).onFalse(intake.getNewSetVoltsCommand(0));
    testcontroller.x().onTrue(intakeExtender.getNewIntakeExtenderTurnCommand(setIntakeExtenderAngle)).onFalse(intakeExtender.getNewIntakeExtenderTurnCommand(0));
    
  }

  
  // TESTING PR COMMIT
  
  // TESTING PR COMMIT

  
  // TESTING PR COMMIT

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoCommandManager.getAutonomousCommand();
    // Turn off updating odometry based on Apriltags
    vision.disableUpdateOdometryBasedOnApriltags();
    if (autoCommand != null) {
      // Tell vision autonomous path was executed, so pose was set
      vision.updateAutonomous();
    }
    return autoCommand;
  }

  public void teleopInit() {
    if (!this.m_TeleopInitialized) {
      // Only want to initialize starting position once (if teleop multiple times dont reset pose
      // again)
      vision.updateStartingPosition();
      // Turn on updating odometry based on Apriltags
      vision.enableUpdateOdometryBasedOnApriltags();
      m_TeleopInitialized = true;
      SignalLogger.setPath("/media/sda1/");
      SignalLogger.start();
    }
  }
}

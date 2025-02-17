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

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.BargeScore;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GroundIntakeToStow;
import frc.robot.commands.L2.StowToL2;
import frc.robot.commands.L2.TakeAlgaeL2;
import frc.robot.commands.OutakeAlgae;
import frc.robot.commands.OutakeCoral;
import frc.robot.commands.StowCommand;
import frc.robot.commands.StowToAlgaeStow;
import frc.robot.commands.StowToGroundIntake;
import frc.robot.commands.StowToL1;
import frc.robot.commands.StowToL3;
import frc.robot.commands.StowToL4;
import frc.robot.commands.TakeAlgaeL3;
import frc.robot.commands.TakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffectorIOSim;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffectorIOTalonFX;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.arm.ArmJointIOSim;
import frc.robot.subsystems.arm.ArmJointIOTalonFX;
import frc.robot.subsystems.arm.constants.ElbowConstants;
import frc.robot.subsystems.arm.constants.ShoulderConstants;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorIOSim;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.intakeextender.IntakeExtenderIOSim;
import frc.robot.subsystems.intakeextender.IntakeExtenderIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import static frc.robot.subsystems.vision.VisionConstants.limelightBackName;
import static frc.robot.subsystems.vision.VisionConstants.limelightFrontName;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraBack;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraFront;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;
import frc.robot.util.ReefPositionsUtil.DeAlgaeLevel;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final double DRIVE_SPEED = 1.0;
  private final double ANGULAR_SPEED = 0.75;
  private final Wrist wrist;

  private final ArmJoint shoulder;
  private final ArmJoint elbow;

  private final Elevator elevator;

  private final CoralEndEffector coralEndEffector;

  private final Intake intake;

  private final AlgaeEndEffector algaeEndEffector;

  private final IntakeExtender intakeExtender;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController co_controller = new CommandXboxController(1);
  private final CommandXboxController characterizeController = new CommandXboxController(2);
  private final CommandXboxController testcontroller = new CommandXboxController(3);

  private final AprilTagVision vision;

  private AutoCommandManager autoCommandManager;
  private RobotState robotState;
  private ReefPositionsUtil reefPositions;

  private boolean m_TeleopInitialized = false;

  final LoggedTunableNumber setElevatorDistance = new LoggedTunableNumber("RobotState/Elevator/setDistance", 58);
  final LoggedTunableNumber setWristAngle = new LoggedTunableNumber("RobotState/Wrist/setAngle", 90);
  final LoggedTunableNumber setToesiesVolts = new LoggedTunableNumber("RobotState/Toesies/setVolts", 2);
  final LoggedTunableNumber setFingeysVolts = new LoggedTunableNumber("RobotState/Fingeys/setVolts", 2);
  final LoggedTunableNumber setIntakeExtenderAngle = new LoggedTunableNumber("RobotState/IntakeExtender/setAngle", 90);
  final LoggedTunableNumber setIntakeVolts = new LoggedTunableNumber("RobotState/Intake/setVolts", 4);
  final LoggedTunableNumber setShoulderAngle = new LoggedTunableNumber("RobotState/Shoulder/setAngle", 0);
  final LoggedTunableNumber setElbowAngle = new LoggedTunableNumber("RobotState/Elbow/setAngle", 0);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){

    CanDef.Builder canivoreCanBuilder = CanDef.builder().bus(CanBus.CANivore);
    CanDef.Builder rioCanBuilder = CanDef.builder().bus(CanBus.Rio);

    switch (Constants.currentMode) {
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
        elevator = new Elevator(
          new ElevatorIOSim(
            4,
            new ElevatorSim(
              LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60Foc(2), 
                Pounds.of(45).in(Kilograms),
                Inches.of(Elevator.SPOOL_RADIUS).in(Meters), 
                Elevator.REDUCTION
              ), 
              DCMotor.getKrakenX60Foc(2), 
              Inches.of(0).in(Meters),
              Inches.of(32).in(Meters),
              true, 
              Inches.of(0).in(Meters)
            )
          )
        );

        shoulder = new ArmJoint(new ArmJointIOSim(new ShoulderConstants()),Optional.empty());
        elbow = new ArmJoint(new ArmJointIOSim(new ElbowConstants()),Optional.of(shoulder));
        coralEndEffector = new CoralEndEffector(new CoralEndEffectorIOSim(121));
        intake = new Intake(new IntakeIOSim(15));
        algaeEndEffector = new AlgaeEndEffector(new AlgaeEndEffectorIOSim(12));
        intakeExtender = new IntakeExtender( new IntakeExtenderIOSim(16));
        
      break;
      
      //real is default because it is safer
      default:
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

        wrist = new Wrist(new WristIOTalonFX(canivoreCanBuilder.id(11).build(),canivoreCanBuilder.id(15).build()));

        elevator = new Elevator(new ElevatorIOTalonFX(rioCanBuilder.id(13).build(),rioCanBuilder.id(14).build()));

        shoulder = new ArmJoint(new ArmJointIOTalonFX(new ShoulderConstants(), InvertedValue.CounterClockwise_Positive), Optional.empty());
        elbow = new ArmJoint(new ArmJointIOTalonFX(new ElbowConstants(), InvertedValue.CounterClockwise_Positive), Optional.of(shoulder));

        coralEndEffector = new CoralEndEffector(new CoralEndEffectorIOTalonFX(canivoreCanBuilder.id(12).build()));
        
        intake = new Intake(new IntakeIOTalonFX(canivoreCanBuilder.id(18).build(),canivoreCanBuilder.id(17).build()));

        intakeExtender = new IntakeExtender(new IntakeExtenderIOTalonFX(rioCanBuilder.id(16).build()));

        algaeEndEffector = new AlgaeEndEffector(new AlgaeEndEffectorIOTalonFX(canivoreCanBuilder.id(15).build()));

        // vision =
        //     new Vision(
        //         demoDrive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));
        // arm = new ArmJoint(new ArmJointIOTalonFX(), null);


        // Real robot, instantiate hardware IO implementations
        break;

      // default:
      //   drive =
      //       new Drive(
      //           new GyroIO() {},
      //           new ModuleIO() {},
      //           new ModuleIO() {},
      //           new ModuleIO() {},
      //           new ModuleIO() {});

      //   // Replayed robot, disable IO implementations
      //   // (Use same number of dummy implementations as the real robot)
      //   vision =
      //       new AprilTagVision(
      //           drive::setPose, drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

      //   wrist = null;
      //   elevator = null;
      //   shoulder = null;
      //   elbow = null;
      //   fingeys = null;
      //   intake = null;
      //   algaeEndEffector = null;
      //   intakeExtender = null;

      //   throw new Exception("The robot is in neither sim nor real. Something has gone seriously wrong");
    }


    autoCommandManager = new AutoCommandManager(drive, shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector);
    reefPositions = ReefPositionsUtil.getInstance();

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

    //Scoring
      //Constants
    final double L4_READY_POS = -100;
    final double L3_READY_POS = 50;
    final double L2_READY_POS = 50;

      //L4
    controller.rightTrigger().and(co_controller.y())
      .onTrue(elbow.getNewSetAngleCommand(L4_READY_POS-80).alongWith(wrist.getNewApplyCoastModeCommand()).alongWith(new WaitCommand(0.5)).andThen(coralEndEffector.getNewSetVoltsCommand(-4)))
      .onFalse(coralEndEffector.getNewSetVoltsCommand(1).alongWith(elbow.getNewSetAngleCommand(L4_READY_POS)).alongWith(new WaitCommand(0.2)).andThen(wrist.getNewWristTurnCommand(0)));
      //L3
    controller.rightTrigger().and(co_controller.x())
      .onTrue(elbow.getNewSetAngleCommand(L3_READY_POS-80).alongWith(wrist.getNewApplyCoastModeCommand()).alongWith(new WaitCommand(0.5)).andThen(coralEndEffector.getNewSetVoltsCommand(-4)))
      .onFalse(coralEndEffector.getNewSetVoltsCommand(1).alongWith(elbow.getNewSetAngleCommand(L3_READY_POS)).alongWith(new WaitCommand(0.2)).andThen(wrist.getNewWristTurnCommand(0)));
      //L2
    controller.rightTrigger().and(co_controller.b())
      .onTrue(elbow.getNewSetAngleCommand(L2_READY_POS-80).alongWith(wrist.getNewApplyCoastModeCommand()).alongWith(new WaitCommand(0.5)).andThen(coralEndEffector.getNewSetVoltsCommand(-4)))
      .onFalse(coralEndEffector.getNewSetVoltsCommand(1).alongWith(elbow.getNewSetAngleCommand(L2_READY_POS)).alongWith(new WaitCommand(0.2)).andThen(wrist.getNewWristTurnCommand(0)));
      //L1
    controller.rightTrigger().and(co_controller.a())
      .onTrue(coralEndEffector.getNewSetVoltsCommand(-4))
      .onFalse(coralEndEffector.getNewSetVoltsCommand(1));
      //Barge
    // controller.rightTrigger().and(co_controller.povUp())
    //   .onTrue(algaeEndEffector.getNewSetVoltsCommand(-6.0))
    //   .onFalse(algaeEndEffector.getNewSetVoltsCommand(0.0));

    // co_controller.y().onTrue(reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L4)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    // co_controller.x().onTrue(reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L3)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    // co_controller.b().onTrue(reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L2)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    // co_controller.a().onTrue(reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L1)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector)); // Trough

    //L4
    co_controller.y().onTrue(new StowToL4(shoulder, elbow, elevator, wrist, coralEndEffector)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    //L3
    co_controller.x().onTrue(new StowToL3(shoulder, elbow, wrist, coralEndEffector, elevator)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    //L2
    co_controller.b().onTrue(new StowToL2(shoulder, elbow, elevator, wrist)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    //L1
    co_controller.a().onTrue(new StowToL1(shoulder, elbow, wrist, coralEndEffector)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    //Barge
    // co_controller.povUp().onTrue(new BargeScore(shoulder, elbow, elevator, wrist)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));


    //Intake
    controller.leftBumper()
      .onTrue(new TakeCoral(shoulder, elbow, elevator, wrist, coralEndEffector))
      .onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));

    // Reef DeAlgaefy scoring position sets
    // co_controller.rightBumper().onTrue(reefPositions.getNewSetDeAlgaeLevel(DeAlgaeLevel.Top)); // L3/4
    // co_controller.rightTrigger().onTrue(reefPositions.getNewSetDeAlgaeLevel(DeAlgaeLevel.Low)); // L2/3

    //Algae Commands
      //Take Algae L2
    co_controller.rightTrigger()
      .onTrue(new TakeAlgaeL2(shoulder, elbow, wrist, algaeEndEffector, elevator))
      .onFalse(algaeEndEffector.getNewSetVoltsCommand(4).alongWith(elevator.getNewSetDistanceCommand(0)).alongWith(elbow.getNewSetAngleCommand(70)).alongWith(shoulder.getNewSetAngleCommand(20)));
      //Take Algae L3
    co_controller.rightBumper()
      .onTrue(new TakeAlgaeL3(shoulder, elbow, wrist, algaeEndEffector, elevator))
      .onFalse(algaeEndEffector.getNewSetVoltsCommand(4).alongWith(elevator.getNewSetDistanceCommand(0)).alongWith(elbow.getNewSetAngleCommand(70)).alongWith(shoulder.getNewSetAngleCommand(20)));
      //Eject Algae
    co_controller.leftTrigger().onTrue(new OutakeAlgae(algaeEndEffector)).onFalse(algaeEndEffector.getNewSetVoltsCommand(0));

    // TODO: Implement climbing controls (L Bumper climb and (maybe) L Trigger unclimb)

    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));
    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

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
    
    // // controller.rightBumper()
    // // .onTrue(
    // //   elbow.getNewSetAngleCommand(-30).alongWith(shoulder.getNewSetAngleCommand(75))
    // //   .andThen(new WaitUntilCommand(elbow.getNewAtSetpointTrigger().and(shoulder.getNewAtSetpointTrigger())))
    // //   .andThen(
    // //     elbow.getNewSetAngleCommand(70)
    // //     .until(elbow.getNewAtSetpointTrigger().and(shoulder.getNewAtSetpointTrigger()))
    // //   )
    
    // // ).onFalse(elbow.getNewSetAngleCommand(0).alongWith(shoulder.getNewSetAngleCommand(0)));

    // // Auto aim command example FOR DIFFERENTIAL DRIVE
    // // @SuppressWarnings("resource")
    // // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // // keyboard
    // //     .button(1)
    // //     .whileTrue(
    // //         Commands.startRun(
    // //             () -> {
    // //               aimController.reset();
    // //             },
    // //             () -> {
    // //               drive.run(0.0, aimController.calculate(vision.getTargetX(0).getRadians()));
    // //             },
    // //             drive));
  }

  public Command TEMPgetStowCommand() {
    return shoulder.getNewSetAngleCommand(68).alongWith(elbow.getNewSetAngleCommand(65)).alongWith(wrist.getNewWristTurnCommand(0)).alongWith(elevator.getNewSetDistanceCommand(0));
  }

  public void configureTestButtonBindings (){
    // testcontroller.leftBumper().onTrue(wrist.getNewWristTurnCommand(setWristAngle)).onFalse(wrist.getNewWristTurnCommand(0));
    // testcontroller.rightBumper().onTrue(toesies.getNewSetVoltsCommand(setToesiesVolts)).onFalse(toesies.getNewSetVoltsCommand(0));
    // testcontroller.leftTrigger().onTrue(fingeys.getNewSetVoltsCommand(setFingeysVolts)).onFalse(fingeys.getNewSetVoltsCommand(0));
    // testcontroller.rightTrigger().onTrue(intake.getNewSetVoltsCommand(setIntakeVolts)).onFalse(intake.getNewSetVoltsCommand(0));
    // testcontroller.x().onTrue(intakeExtender.getNewIntakeExtenderTurnCommand(setIntakeExtenderAngle)).onFalse(intakeExtender.getNewIntakeExtenderTurnCommand(0));
    // testcontroller.a().onTrue(shoulder.getNewSetAngleCommand(setShoulderAngle)).onFalse(shoulder.getNewSetAngleCommand(90));
    // testcontroller.b().onTrue(elbow.getNewSetAngleCommand(setElbowAngle)).onFalse(elbow.getNewSetAngleCommand(0));
    // testcontroller.rightBumper().onTrue(new StowToL2(shoulder, elbow, wrist, coralEndEffector)).onFalse(TEMPgetStowCommand());
    // controller.a().whileTrue(elbow.getNewSetAngleCommand(10).alongWith(new WaitCommand(0.5)).andThen(coralEndEffector.getNewSetVoltsCommand(-4))).onFalse(coralEndEffector.getNewSetVoltsCommand(0)).onFalse(TEMPgetStowCommand());
    // testcontroller.povRight().whileTrue(new TakeCoral(shoulder, elbow, elevator, wrist, coralEndEffector)).onFalse(coralEndEffector.getNewSetVoltsCommand(0)).onFalse(TEMPgetStowCommand());
    // // controller.leftBumper().onTrue(new StowToL3(shoulder, elbow, wrist, coralEndEffector, elevator)).onFalse(TEMPgetStowCommand());
    // testcontroller.leftTrigger().onTrue(new TakeAlgaeL2(shoulder, elbow, wrist, algaeEndEffector, elevator)).onFalse(algaeEndEffector.getNewSetVoltsCommand(4).alongWith(elevator.getNewSetDistanceCommand(0)));
    // testcontroller.x().onTrue(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    testcontroller.leftBumper().onTrue(new OutakeAlgae(algaeEndEffector)).onFalse(algaeEndEffector.getNewSetVoltsCommand(0));
    testcontroller.rightBumper().onTrue(new OutakeCoral(coralEndEffector)).onFalse(coralEndEffector.getNewSetVoltsCommand(0));
    // testcontroller.y().onTrue(new TakeCoral(shoulder, elbow, elevator, wrist)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    // testcontroller.povDown().onTrue(new BargeScore(shoulder, elbow, elevator, wrist, coralEndEffector)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));

    
    SmartDashboard.putData(new GroundIntakeToStow(shoulder, elbow, wrist, coralEndEffector));
    SmartDashboard.putData(new StowToGroundIntake(shoulder, elbow, wrist, coralEndEffector));
    SmartDashboard.putData(new StowToAlgaeStow(shoulder, elbow, wrist, coralEndEffector));
    SmartDashboard.putData(new StowToL3(shoulder, elbow, wrist, coralEndEffector, elevator));
    SmartDashboard.putData(new StowToL4(shoulder, elbow, elevator, wrist, coralEndEffector));
    SmartDashboard.putData(new TakeAlgaeL2(shoulder, elbow, wrist, algaeEndEffector, elevator));
    SmartDashboard.putData(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
  }
  
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

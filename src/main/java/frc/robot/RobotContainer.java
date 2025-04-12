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

import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.google.gson.annotations.Until;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlgaeStowCommand;
import frc.robot.commands.BargeAlignCommand;
import frc.robot.commands.BargeScoreThrowCommand;
import frc.robot.commands.DisengageClimber;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.EngageClimber;
import frc.robot.commands.L4ToStow;
import frc.robot.commands.NeutralClimber;
import frc.robot.commands.OutakeAlgae;
import frc.robot.commands.OutakeCoral;
import frc.robot.commands.ProcessorAlignCommand;
import frc.robot.commands.ReadyProcessorScore;
import frc.robot.commands.ReefScoreCommandFactory;
import frc.robot.commands.ReefScoreCommandFactory.ReefPosition;
import frc.robot.commands.StationIntakeCommand;
import frc.robot.commands.StationIntakeCommandFactory;
import frc.robot.commands.StationIntakeCommandFactory.IntakePosition;
import frc.robot.commands.StationIntakeToStow;
import frc.robot.commands.StowCommand;
import frc.robot.commands.StowToBarge;
import frc.robot.commands.StowToL3;
import frc.robot.commands.StowToL4;
import frc.robot.commands.TakeAlgaeL2;
import frc.robot.commands.TakeAlgaeL3;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffectorIOSim;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffectorIOTalonFX;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.arm.ArmJointIOSim;
import frc.robot.subsystems.arm.ArmJointIOTalonFX;
import frc.robot.subsystems.arm.constants.ElbowConstants;
import frc.robot.subsystems.arm.constants.ShoulderConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
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
import frc.robot.subsystems.led.CandleSubsystem;
import frc.robot.subsystems.led.CandleSubsystem.AnimationTypes;
import frc.robot.subsystems.vision.AprilTagVision;
import static frc.robot.subsystems.vision.VisionConstants.limelightLeftName;
import static frc.robot.subsystems.vision.VisionConstants.limelightRightName;
import static frc.robot.subsystems.vision.VisionConstants.limelightFrontName;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraLeft;
import static frc.robot.subsystems.vision.VisionConstants.robotToCameraRight;
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
import frc.robot.util.ReefPositionsUtil.AutoAlignSide;
import frc.robot.util.ReefPositionsUtil.DeAlgaeLevel;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;
import frc.robot.util.SelectorCommandFactory;

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
  private final boolean INVERT_ENDGAME = true;
  private final Wrist wrist;

  private final ArmJoint shoulder;
  private final ArmJoint elbow;

  private final Elevator elevator;

  private final CoralEndEffector coralEndEffector;

  private final AlgaeEndEffector algaeEndEffector;

  private final Climber climber;

  private final CandleSubsystem LEDs1;

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
  final LoggedTunableNumber setWristAngle = new LoggedTunableNumber("RobotState/Wrist/setAngle", -90);
  final LoggedTunableNumber setToesiesVolts = new LoggedTunableNumber("RobotState/Toesies/setVolts", 2);
  final LoggedTunableNumber setFingeysVolts = new LoggedTunableNumber("RobotState/Fingeys/setVolts", 2);
  final LoggedTunableNumber setIntakeVolts = new LoggedTunableNumber("RobotState/Intake/setVolts", 4);
  final LoggedTunableNumber setShoulderAngle = new LoggedTunableNumber("RobotState/Shoulder/setAngle", 0);
  final LoggedTunableNumber setElbowAngle = new LoggedTunableNumber("RobotState/Elbow/setAngle", 0);
  final LoggedTunableNumber setClimberVolts = new LoggedTunableNumber("RobotState/Climber/setVolts", 4);

  private LoggedDashboardChooser<IntakePosition> intakePosChooser = new LoggedDashboardChooser<>("Intake Position");
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){

    // Intake Options
    intakePosChooser.addDefaultOption("Inside", IntakePosition.Inside);
    intakePosChooser.addOption("Center", IntakePosition.Center);
    intakePosChooser.addOption("Outside", IntakePosition.Outside);

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
                drive::addVisionMeasurementAutoAlign,
                new VisionIOPhotonVisionSim(limelightLeftName, robotToCameraLeft, drive::getPose),
                new VisionIOPhotonVisionSim(limelightRightName, robotToCameraRight, drive::getPose),
                new VisionIOPhotonVisionSim(limelightFrontName, robotToCameraFront, drive::getPose));

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
        algaeEndEffector = new AlgaeEndEffector(new AlgaeEndEffectorIOSim(12));
        climber = new Climber(new ClimberIOSim(19));
        
        SmartDashboard.putData(drive);
      break;
      
      //real is default because it is safer
      default:
      //TODO: add back in dummy drive for replay and add REAL case back
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
                drive::addVisionMeasurementAutoAlign,
                new VisionIOLimelight(limelightLeftName, drive::getRotation),
                new VisionIOLimelight(limelightRightName, drive::getRotation),
                new VisionIOLimelight(limelightFrontName, drive::getRotation));

        wrist = new Wrist(new WristIOTalonFX(canivoreCanBuilder.id(11).build(),canivoreCanBuilder.id(15).build()));

        elevator = new Elevator(new ElevatorIOTalonFX(rioCanBuilder.id(13).build(),rioCanBuilder.id(14).build()));

        shoulder = new ArmJoint(new ArmJointIOTalonFX(new ShoulderConstants(), InvertedValue.CounterClockwise_Positive, SensorDirectionValue.Clockwise_Positive, NeutralModeValue.Brake), Optional.empty());
        elbow = new ArmJoint(new ArmJointIOTalonFX(new ElbowConstants(), InvertedValue.CounterClockwise_Positive, SensorDirectionValue.Clockwise_Positive, NeutralModeValue.Brake), Optional.of(shoulder));

        coralEndEffector = new CoralEndEffector(new CoralEndEffectorIOTalonFX(canivoreCanBuilder.id(12).build(), canivoreCanBuilder.id(17).build()));
        // coralEndEffector = new CoralEndEffector(new CoralEndEffectorIONova(rioCanBuilder.id(12).build(), canivoreCanBuilder.id(17).build()));

        algaeEndEffector = new AlgaeEndEffector(new AlgaeEndEffectorIOTalonFX(canivoreCanBuilder.id(15).build(), canivoreCanBuilder.id(24).build()));
        // algaeEndEffector = new AlgaeEndEffector(new AlgaeEndEffectorIONova(rioCanBuilder.id(15).build(), canivoreCanBuilder.id(24).build()));

        climber = new Climber(new ClimberIOTalonFX(rioCanBuilder.id(19).build(), INVERT_ENDGAME));

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

      //   throw new Exception("The robot is in neither sim nor real. Something has gone seriously wrong");
    }


    autoCommandManager = new AutoCommandManager(drive, shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector);
    reefPositions = ReefPositionsUtil.getInstance();
    ReefScoreCommandFactory.initialize();
    LEDs1 = new CandleSubsystem(rioCanBuilder.id(28).build(),30);

    // Configure the button bindings
    configureDriverBindings();
    configureTestButtonBindings();
    configureCharacterizationButtonBindings();
  }

  public void configureDriverBindings() {

    //#region controller

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
          drive,
          () -> -controller.getLeftY() * DRIVE_SPEED,
          () -> -controller.getLeftX() * DRIVE_SPEED,
          () -> -controller.getRightX() * ANGULAR_SPEED
        )
    );
    
    // Conditional DeAlgae no auto align
    controller.leftTrigger()
    .and(()->!ReefPositionsUtil.getInstance().getIsAutoAligning())
    .onTrue(new ConditionalCommand(
      new TakeAlgaeL2(shoulder, elbow, wrist, algaeEndEffector, elevator), 
      new TakeAlgaeL3(shoulder, elbow, wrist, algaeEndEffector, elevator), 
      () -> reefPositions.isSelected(DeAlgaeLevel.Low)))
    .onFalse(new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector));

    //conditional DeAlgae auto align
    controller.leftTrigger()
    .and(()->ReefPositionsUtil.getInstance().getIsAutoAligning())
    .onTrue(new ConditionalCommand(
      ReefScoreCommandFactory.getNewAlgaePluckAutoAlignSequenceCommand(DeAlgaeLevel.Low, drive, shoulder, elbow, elevator, wrist, algaeEndEffector), 
      ReefScoreCommandFactory.getNewAlgaePluckAutoAlignSequenceCommand(DeAlgaeLevel.Top, drive, shoulder, elbow, elevator, wrist, algaeEndEffector), 
      () -> reefPositions.isSelected(DeAlgaeLevel.Low)))
    .onFalse(new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector));

    // Coral Station Intake Auto Align Sequence†
    // controller.leftBumper()
    //   .and(() -> ReefPositionsUtil.getInstance().getIsAutoAligning())
    //   .and(coralEndEffector.hasCoralTrigger().negate())
    //   .and(algaeEndEffector.hasAlgaeTrigger().negate()) 
    //   .onTrue(
    //     StationIntakeCommandFactory.getNewStationIntakeSequence(
    //         () -> {
    //         IntakePosition pos = intakePosChooser.get();
    //         return pos == null ? IntakePosition.Inside : pos;
    //       },
    //         shoulder, elbow, elevator, wrist, coralEndEffector, drive
    //     )
    //   )
    // .onFalse(new StationIntakeToStow(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));

    // Go to barge auto align
    controller.y()
      .and(()->ReefPositionsUtil.getInstance().getIsAutoAligning())
      .whileTrue(
        DriveCommands.brakeDrive(drive)
          .alongWith(new StowToBarge(shoulder, elbow, wrist))
          .andThen(new BargeAlignCommand(drive,()->MathUtil.applyDeadband(controller.getLeftX(),0.1)))
          .andThen(new BargeScoreThrowCommand(elevator, wrist, algaeEndEffector))
          .andThen(new WaitCommand(0.5))
          .andThen(algaeEndEffector.getNewSetVoltsCommand(0.0))
          .andThen(elevator.getNewSetDistanceCommand(0.0))
          .andThen(
            new ConditionalCommand(
              new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector), 
              new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
              algaeEndEffector.hasAlgaeTrigger()
            ).onlyIf(controller.y().negate())
          )
      )
      .onFalse(
        new ConditionalCommand(
          new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector), 
          new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
          algaeEndEffector.hasAlgaeTrigger()
        )
      );

    // Go to barge no auto align
    controller.y()
      .and(()->!ReefPositionsUtil.getInstance().getIsAutoAligning())
      .onTrue(
        new StowToBarge(shoulder, elbow, wrist)
      )
      .onFalse(
        new ConditionalCommand(
          new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector), 
          new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
          algaeEndEffector.hasAlgaeTrigger()
        )
      );

    // Processer Score
    controller.b()
      .and(()->ReefPositionsUtil.getInstance().getIsAutoAligning())
      .onTrue(
        new ReadyProcessorScore(shoulder, elbow, elevator, wrist, algaeEndEffector)
          .alongWith(new ProcessorAlignCommand(drive))
        .andThen(new OutakeAlgae(algaeEndEffector))
        .andThen(new WaitUntilCommand(algaeEndEffector.hasAlgaeTrigger().negate()))
        .andThen(
          new ConditionalCommand(
            new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector), 
            new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
            algaeEndEffector.hasAlgaeTrigger()
          )
        )
      );

    controller.b()
      .and(()->!ReefPositionsUtil.getInstance().getIsAutoAligning())
      .onTrue(
        new ReadyProcessorScore(shoulder, elbow, elevator, wrist, algaeEndEffector)
        )
      .onFalse(
        new OutakeAlgae(algaeEndEffector)
        .andThen(new WaitCommand(0.3))
        .andThen(new WaitUntilCommand(algaeEndEffector.hasAlgaeTrigger().negate()))
        .andThen(
          new ConditionalCommand(
            new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector), 
            new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
            algaeEndEffector.hasAlgaeTrigger()
          )
        )
      );
    
    //#region reef scoring

    // Hashmaps for Coral level commands

    Map<ReefPositionsUtil.ScoreLevel,Command> coralLevelCommands = SelectorCommandFactory.getCoralLevelPrepCommandSelector(shoulder, elbow, elevator, wrist);
    Map<ReefPositionsUtil.ScoreLevel,Command> scoreCoralLevelCommands = SelectorCommandFactory.getCoralLevelScoreCommandSelector(shoulder, elbow, elevator, wrist, coralEndEffector);
    Map<ReefPositionsUtil.ScoreLevel,Command> stopCoralLevelCommands = SelectorCommandFactory.getCoralLevelStopScoreCommandSelector(elbow, wrist, coralEndEffector, drive);
    Map<ReefPositionsUtil.ScoreLevel,Command> waitUntilCoralLevelCommands = SelectorCommandFactory.getCoralLevelWaitUntilAtLevelCommandSelector(shoulder, elbow, elevator, wrist);

    /*  1. Right bumper pressed and AutoAligning is disabled
          - Goes to the score level of what is selected by the co-driver
        2. Right bumper pressed and AutoAligning is enabled
          - Nothing happens
        3. Right bumper is released and AutoAligning is disabled
          - If reefScoreL4
            - Goes to Coral Stow
          - Else
            - Goes to Algae Stow
    */

    // Left side reef auto align
    controller.rightBumper()
    .and(
      ()->reefPositions.getIsAutoAligning()
    ).and(
      () -> {return reefPositions.getAutoAlignSide() == AutoAlignSide.Left;}
    ).whileTrue(
      ReefScoreCommandFactory.getNewReefCoralScoreSequence(
        ReefPosition.Left, 
        true,
        SelectorCommandFactory.getCoralLevelPrepCommandSelector(shoulder, elbow, elevator, wrist), 
        SelectorCommandFactory.getCoralLevelScoreCommandSelector(shoulder, elbow, elevator, wrist, coralEndEffector),
        SelectorCommandFactory.getCoralLevelStopScoreCommandSelector(elbow, wrist, coralEndEffector, drive),
        SelectorCommandFactory.getCoralLevelWaitUntilAtLevelCommandSelector(shoulder, elbow, elevator, wrist),
        drive
      )
      .alongWith(LEDs1.getSetLEDColorCommand(new Color8Bit(Color.kGreen)))
    ).onFalse(
      new ConditionalCommand(
        new L4ToStow(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
        new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
        () -> reefPositions.isSelected(ScoreLevel.L4)
      )
      .alongWith(LEDs1.getSetLEDColorCommand(new Color8Bit(Color.kLightBlue)))
    );

    // Right side reef auto align
    controller.rightBumper()
    .and(
      ()->reefPositions.getIsAutoAligning()
    ).and(
      () -> {return reefPositions.getAutoAlignSide() == AutoAlignSide.Right;}
    ).whileTrue(
      ReefScoreCommandFactory.getNewReefCoralScoreSequence(
        ReefPosition.Right, 
        true,
        SelectorCommandFactory.getCoralLevelPrepCommandSelector(shoulder, elbow, elevator, wrist), 
        SelectorCommandFactory.getCoralLevelScoreCommandSelector(shoulder, elbow, elevator, wrist, coralEndEffector),
        SelectorCommandFactory.getCoralLevelStopScoreCommandSelector(elbow, wrist, coralEndEffector, drive),
        SelectorCommandFactory.getCoralLevelWaitUntilAtLevelCommandSelector(shoulder, elbow, elevator, wrist),
        drive)
        .alongWith(LEDs1.getSetLEDColorCommand(new Color8Bit(Color.kGreen)))
    ).onFalse(
      new ConditionalCommand(
        new L4ToStow(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
        new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
        () -> reefPositions.isSelected(ScoreLevel.L4)
      )
      .alongWith(LEDs1.getSetLEDColorCommand(new Color8Bit(Color.kLightBlue)))
    );

    // Right side reef auto align superscore
    controller.leftBumper()
    .and(
      ()->reefPositions.getIsAutoAligning()
    ).and(
      () -> {return reefPositions.getAutoAlignSide() == AutoAlignSide.Right;}
    ).whileTrue(
      ReefScoreCommandFactory.getNewReefCoralScoreSequence(
        ReefPosition.Right, 
        true,
        SelectorCommandFactory.getCoralLevelPrepCommandSelector(shoulder, elbow, elevator, wrist), 
        SelectorCommandFactory.getCoralLevelScoreCommandSelector(shoulder, elbow, elevator, wrist, coralEndEffector),
        SelectorCommandFactory.getCoralLevelStopScoreCommandSelector(elbow, wrist, coralEndEffector, drive),
        SelectorCommandFactory.getCoralLevelWaitUntilAtLevelCommandSelector(shoulder, elbow, elevator, wrist),
        drive)
        .andThen(
          new ConditionalCommand(
            ReefScoreCommandFactory.getNewAlgaePluckAutoAlignSequenceCommand(DeAlgaeLevel.Low, drive, shoulder, elbow, elevator, wrist, algaeEndEffector), 
            ReefScoreCommandFactory.getNewAlgaePluckAutoAlignSequenceCommand(DeAlgaeLevel.Top, drive, shoulder, elbow, elevator, wrist, algaeEndEffector), 
            () -> reefPositions.isSelected(DeAlgaeLevel.Low)))
    ).onFalse(
      new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector)
    );

    controller.leftBumper()
    .and(
      ()->reefPositions.getIsAutoAligning()
    ).and(
      () -> {return reefPositions.getAutoAlignSide() == AutoAlignSide.Left;}
    ).whileTrue(
      ReefScoreCommandFactory.getNewReefCoralScoreSequence(
        ReefPosition.Left, 
        true,
        SelectorCommandFactory.getCoralLevelPrepCommandSelector(shoulder, elbow, elevator, wrist), 
        SelectorCommandFactory.getCoralLevelScoreCommandSelector(shoulder, elbow, elevator, wrist, coralEndEffector),
        SelectorCommandFactory.getCoralLevelStopScoreCommandSelector(elbow, wrist, coralEndEffector, drive),
        SelectorCommandFactory.getCoralLevelWaitUntilAtLevelCommandSelector(shoulder, elbow, elevator, wrist),
        drive)
        .andThen(
          new ConditionalCommand(
            ReefScoreCommandFactory.getNewAlgaePluckAutoAlignSequenceCommand(DeAlgaeLevel.Low, drive, shoulder, elbow, elevator, wrist, algaeEndEffector), 
            ReefScoreCommandFactory.getNewAlgaePluckAutoAlignSequenceCommand(DeAlgaeLevel.Top, drive, shoulder, elbow, elevator, wrist, algaeEndEffector), 
            () -> reefPositions.isSelected(DeAlgaeLevel.Low)))
    ).onFalse(
      new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector)
    );

    // Go to conditional coral level no auto align
    controller.rightBumper().and(()->!ReefPositionsUtil.getInstance().getIsAutoAligning())
    .onTrue(ReefPositionsUtil.getInstance().getCoralLevelSelector(coralLevelCommands))
    .onFalse(new ConditionalCommand(
      new L4ToStow(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
      new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
      () -> reefPositions.isSelected(ScoreLevel.L4)));

    // Conditional Confirm Coral
    controller.rightTrigger().and(controller.rightBumper()).and(()->!ReefPositionsUtil.getInstance().getIsAutoAligning())
      .onTrue(ReefPositionsUtil.getInstance().getCoralLevelSelector(scoreCoralLevelCommands))
      .onFalse(ReefPositionsUtil.getInstance().getCoralLevelSelector(stopCoralLevelCommands).onlyIf(controller.rightBumper()));
    //#endregion

    // Outtake Algae
    controller.povLeft()
      .onTrue(
        new OutakeAlgae(algaeEndEffector)
      ).onFalse(
        algaeEndEffector.getNewSetVoltsCommand(0.0)
      );

    //Outtake Coral
    controller.povRight()
      .onTrue(new OutakeCoral(coralEndEffector))
      .onFalse(coralEndEffector.getNewSetVoltsCommand(0.0));

    // Toggle auto align on/off
    controller.start()
      .onTrue(new InstantCommand(() -> {reefPositions.setIsAutoAligning(!reefPositions.getIsAutoAligning());}));

    //#endregion
    //#region Co-Controller

    // Coral Station Intake No Auto Align
    co_controller.leftBumper()
      .onTrue(
        new StationIntakeCommand(shoulder, elbow, elevator, wrist, coralEndEffector)
      )
      .onFalse(new StationIntakeToStow(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));

    // Select L4
    co_controller.y().and(controller.rightBumper().negate())
      .onTrue(reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L4));
    // Select L3
    co_controller.x().and(controller.rightBumper().negate())
      .onTrue(reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L3));
    // Select L2
    co_controller.b().and(controller.rightBumper().negate())
      .onTrue(reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L2));
    // Select L1
    co_controller.a().and(controller.rightBumper().negate())
      .onTrue(reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L1));

    // Select L3/L4 DeAlgae
    co_controller.rightBumper().and(controller.leftTrigger().negate())
      .onTrue(reefPositions.getNewSetDeAlgaeLevelCommand(DeAlgaeLevel.Top));
    // Select L2/L3 DeAlgae
    co_controller.rightTrigger().and(controller.leftTrigger().negate())
      .onTrue(reefPositions.getNewSetDeAlgaeLevelCommand(DeAlgaeLevel.Low));

    // Back Coral Station Intake
    // co_controller.povUp()
    //   .and(algaeEndEffector.hasAlgaeTrigger().negate())
    //   .onTrue(new StationIntakeReverseCommand(shoulder, elbow, elevator, wrist, coralEndEffector))
    //   .onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));

    // Select Left Auto Aligning
    co_controller.povLeft()
      .onTrue(new InstantCommand(() ->ReefPositionsUtil.getInstance().setAutoAlignSide(ReefPositionsUtil.AutoAlignSide.Left)));

    // Select Right Auto Aligning
    co_controller.povRight()
      .onTrue(new InstantCommand(() ->ReefPositionsUtil.getInstance().setAutoAlignSide(ReefPositionsUtil.AutoAlignSide.Right)));
    
    // Engage climber
    co_controller.start().and(co_controller.back().negate())
    .whileTrue(
      new EngageClimber(climber, shoulder, elbow) 
        .andThen(new WaitUntilCommand(climber.getNewLessThanAngleTrigger(78.0)))
        .andThen(new NeutralClimber(climber, true))
      )
    .onFalse(new NeutralClimber(climber,true ));

    // Disengage climber
    co_controller.back().and(co_controller.start().negate())
    .whileTrue(
      new DisengageClimber(climber, elbow, shoulder)
        .andThen(new WaitUntilCommand(climber.getNewGreaterThanAngleTrigger(160.0)))
        .andThen(new NeutralClimber(climber, false))
    )
    .onFalse(new NeutralClimber(climber,false));

    //#endregion
  }

  public void configureTestButtonBindings (){
    // testcontroller.leftBumper().onTrue(wrist.getNewWristTurnCommand(setWristAngle)).onFalse(wrist.getNewWristTurnCommand(0));
    // testcontroller.rightBumper().onTrue(toesies.getNewSetVoltsCommand(setToesiesVolts)).onFalse(toesies.getNewSetVoltsCommand(0));
    // testcontroller.leftTrigger().onTrue(fingeys.getNewSetVoltsCommand(setFingeysVolts)).onFalse(fingeys.getNewSetVoltsCommand(0));
    // testcontroller.rightTrigger().onTrue(intake.getNewSetVoltsCommand(setIntakeVolts)).onFalse(intake.getNewSetVoltsCommand(0));
    //testcontroller.a().onTrue(shoulder.getNewSetAngleCommand(setShoulderAngle)).onFalse(shoulder.getNewSetAngleCommand(90));
    testcontroller.b().onTrue(elbow.getNewSetAngleCommand(setElbowAngle));/* .onFalse(elbow.getNewSetAngleCommand(0))*/
    // testcontroller.povLeft().onTrue(climber.getNewSetVoltsCommand(setClimberVolts)).onFalse(climber.getNewSetVoltsCommand(0));
    // testcontroller.rightBumper().onTrue(new StowToL2(shoulder, elbow, wrist, coralEndEffector)).onFalse(TEMPgetStowCommand());
    // controller.a().whileTrue(elbow.getNewSetAngleCommand(10).alongWith(new WaitCommand(0.5)).andThen(coralEndEffector.getNewSetVoltsCommand(-4))).onFalse(coralEndEffector.getNewSetVoltsCommand(0)).onFalse(TEMPgetStowCommand());
    // testcontroller.povRight().whileTrue(new TakeCoral(shoulder, elbow, elevator, wrist, coralEndEffector)).onFalse(coralEndEffector.getNewSetVoltsCommand(0)).onFalse(TEMPgetStowCommand());
    // controller.leftBumper().onTrue(new StowToL3(shoulder, elbow, wrist, coralEndEffector, elevator)).onFalse(TEMPgetStowCommand());
    // testcontroller.leftTrigger().onTrue(new TakeAlgaeL2(shoulder, elbow, wrist, algaeEndEffector, elevator)).onFalse(algaeEndEffector.getNewSetVoltsCommand(4).alongWith(elevator.getNewSetDistanceCommand(0)));
    // testcontroller.x().onTrue(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    // testcontroller.leftBumper().onTrue(new OutakeAlgae(algaeEndEffector)).onFalse(algaeEndEffector.getNewSetVoltsCommand(0));
    // testcontroller.rightBumper().onTrue(new OutakeCoral(coralEndEffector)).onFalse(coralEndEffector.getNewSetVoltsCommand(0));
    // testcontroller.y().onTrue(new TakeCoral(shoulder, elbow, elevator, wrist)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    // testcontroller.povDown().onTrue(new BargeScore(shoulder, elbow, elevator, wrist, coralEndEffector)).onFalse(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
    // testcontroller.povLeft().whileTrue(new AutoAlignCommand((thisOneVariableWhichIDontReallyNeedSoIWillUseAnExtremelyConciseNameFor) -> {return new Pose2d(-1, 0, Rotation2d.kCCW_90deg);}, drive));
    // testcontroller.povRight().whileTrue(new AutoAlignCommand((thisOneVariableWhichIDontReallyNeedSoIWillUseAnExtremelyConciseNameFor) -> {return new Pose2d(-1, 0, Rotation2d.kCW_90deg);}, drive));
    // testcontroller.povUp().whileTrue(new AutoAlignCommand((thisOneVariableWhichIDontReallyNeedSoIWillUseAnExtremelyConciseNameFor) -> {return new Pose2d(0, 0, Rotation2d.kZero);}, drive));
    // testcontroller.povDown().whileTrue(
    //   new StowToBarge(shoulder,elbow,elevator,wrist)
    //   .andThen(new BargeAlignCommand(drive,()->testcontroller.getLeftX()))
    //   .andThen(new BargeScoreCommand(algaeEndEffector))
    // ).onFalse(
    //   new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector)
    // );
    testcontroller.povUp().onTrue(
      (new AlgaeStowCommand(shoulder,elbow,elevator,wrist,algaeEndEffector)
        .alongWith(new ProcessorAlignCommand(drive))
      )
      .andThen(new OutakeAlgae(algaeEndEffector))
      .andThen(new WaitCommand(0.3))
      .andThen(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector))
    );

    //TODO AUTO ALIGN AUTO
    //1. Scoring position reached before movement
    //2. Outside field auto align
    //3. Rotates right away, could hit reef
    //4. Targeting into station
    testcontroller.povDown().onTrue(
      reefPositions.getNewSetScoreLevelCommand(ScoreLevel.L4)
      .andThen(ReefScoreCommandFactory.getNewReefCoralScoreSequence(ReefPosition.Left, true, drive, shoulder, elbow, elevator, wrist, coralEndEffector))
      .andThen(
        new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
        StationIntakeCommandFactory.getNewStationIntakeSequence(()->IntakePosition.Center, shoulder, elbow, elevator, wrist, coralEndEffector, drive),
        new WaitUntilCommand(coralEndEffector.hasCoralTrigger()),
        new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
        ReefScoreCommandFactory.getNewReefCoralScoreSequence(ReefPosition.Left, true, drive, shoulder, elbow, elevator, wrist, coralEndEffector),
        new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
        StationIntakeCommandFactory.getNewStationIntakeSequence(()->IntakePosition.Center, shoulder, elbow, elevator, wrist, coralEndEffector, drive),
        new WaitUntilCommand(coralEndEffector.hasCoralTrigger()),
        new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector),
        ReefScoreCommandFactory.getNewReefCoralScoreSequence(ReefPosition.Right, true, drive, shoulder, elbow, elevator, wrist, coralEndEffector),
        new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector)
        )
    );

    testcontroller.start()
      .onTrue(LEDs1.getSetLEDColorCommand(new Color8Bit(Color.kGreen)))
      .onFalse(LEDs1.getSetLEDColorCommand(new Color8Bit(Color.kLightBlue)));
    // testcontroller.b().onTrue(
    //     ReefScoreCommandFactory.getNewAlgaePluckAutoAlignSequenceCommand(DeAlgaeLevel.Low, drive, shoulder, elbow, elevator, wrist, algaeEndEffector))
    //   .onFalse(
    //     new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector)  
    //   );

    SmartDashboard.putData(new AlgaeStowCommand(shoulder, elbow, elevator, wrist, algaeEndEffector));
    SmartDashboard.putData(new StowToL3(shoulder, elbow, wrist, elevator));
    SmartDashboard.putData(new StowToL4(shoulder, elbow, elevator, wrist));
    SmartDashboard.putData(new TakeAlgaeL2(shoulder, elbow, wrist, algaeEndEffector, elevator));
    SmartDashboard.putData(new StowCommand(shoulder, elbow, elevator, wrist, coralEndEffector, algaeEndEffector));
  }

  public void configureCharacterizationButtonBindings() {
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

    characterizeController.povUp()
      .whileTrue(DriveCommands.wheelRadiusCharacterization(drive))
      .onFalse(DriveCommands.brakeDrive(drive));

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
    
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = autoCommandManager.getAutonomousCommand();
    // Turn off updating odometry based on Apriltags
    vision.enableUpdateOdometryBasedOnApriltags();
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

  public void loggingPeriodic() {
    Logger.recordOutput("ReefPositions/Selected Score Position", reefPositions.getScoreLevel());
    Logger.recordOutput("ReefPositions/Selected Auto Align Side", reefPositions.getAutoAlignSide());
    Logger.recordOutput("ReefPositions/Selected DeAlgae Position", reefPositions.getDeAlgaeLevel());
    Logger.recordOutput("ReefPositions/ScorePos/L1", reefPositions.isSelected(ScoreLevel.L1));
    Logger.recordOutput("ReefPositions/ScorePos/L2", reefPositions.isSelected(ScoreLevel.L2));
    Logger.recordOutput("ReefPositions/ScorePos/L3", reefPositions.isSelected(ScoreLevel.L3));
    Logger.recordOutput("ReefPositions/ScorePos/L4", reefPositions.isSelected(ScoreLevel.L4));
    Logger.recordOutput("ReefPositions/AutoAlignSide/Left", reefPositions.isSelected(AutoAlignSide.Left));
    Logger.recordOutput("ReefPositions/AutoAlignSide/Right", reefPositions.isSelected(AutoAlignSide.Right));
    Logger.recordOutput("ReefPositions/DeAlgaePos/Top", reefPositions.isSelected(DeAlgaeLevel.Top));
    Logger.recordOutput("ReefPositions/DeAlgaePos/Low", reefPositions.isSelected(DeAlgaeLevel.Low));
    Logger.recordOutput("ReefPositions/isAutoAlignEnabled", reefPositions.getIsAutoAligning());
  }

  public void disabledPeriodic() {
    // climber.setServoTarget(Degrees.of(180.0));
  }
}

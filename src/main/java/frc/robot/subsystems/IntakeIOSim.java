package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.ReefPositionsUtil.ScoreLevel;

// Intake sim stuff for Maple-Sim
public class IntakeIOSim {
    private final IntakeSimulation intakeSimulation;
    // Talk
    // Configures the bumper size and intake settings
    public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
        // Here, create the intake simulation with respect to the intake on your real robot
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Coral",
            // Specify the drivetrain to which this intake is attached
            driveTrain,
            // Width of the intake
            Meters.of(0.6),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            // The intake can hold up to 1 gamepiece
            1);
    }
    
    // Talk
    public void setRunning(boolean runIntake) {
        if (runIntake)
            intakeSimulation.startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with game pieces
        else
            intakeSimulation.stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }

    public Command setRunnningCommand(boolean runIntake) {
      return new InstantCommand(
        () -> {
          setRunning(runIntake);
        });
    }

    // Talk
    public boolean isGamePieceInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }

    public int getGamePiecesAmount() {
        return intakeSimulation.getGamePiecesAmount();
    }

    // Talk
    public void dropCoral(){
        // final LoggedDashboardChooser<IntakePosition> intakePosChooser = new LoggedDashboardChooser<>("Intake Position");
        // IntakePosition pos = intakePosChooser.get();
        SimulatedArena.getInstance()
            .addGamePieceProjectile(ReefscapeCoralOnFly.DropFromCoralStation(
                ReefscapeCoralOnFly.CoralStationsSide.LEFT_STATION, DriverStation.Alliance.Red, true));
    }

    public Command dropCoralCommand(){
        return new InstantCommand(
        () -> {
          dropCoral();
        });
    }
    
    // Talk
    // Scores coral for each level
    public void scoreCoral(ScoreLevel level, AbstractDriveTrainSimulation driveSimulation) {
        if (intakeSimulation.getGamePiecesAmount() > 0) {
            intakeSimulation.obtainGamePieceFromIntake();
            // Score Level L1 Code
            if (level == ScoreLevel.L1) {
                SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                // Obtain robot position from drive simulation
                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                new Translation2d(-1, 0),
                // Obtain robot speed from drive simulation
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                // Obtain robot facing from drive simulation
                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                // The height at which the coral is ejected
                Meters.of(0.5),
                // The initial speed of the coral
                MetersPerSecond.of(3),
                // The coral is ejected at a 35-degree slope
                Degrees.of(35)));
            }
            // Score Level L2 Code
            if (level == ScoreLevel.L2) {
                SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                // Obtain robot position from drive simulation
                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                new Translation2d(-1.1, 0),
                // Obtain robot speed from drive simulation
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                // Obtain robot facing from drive simulation
                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                // The height at which the coral is ejected
                Meters.of(0.63),
                // The initial speed of the coral
                MetersPerSecond.of(2.5),
                // The coral is ejected at a 35-degree slope
                Degrees.of(35)));
            }
            // Score Level L3 Code
            if (level == ScoreLevel.L3) {
                SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                // Obtain robot position from drive simulation
                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                new Translation2d(-1, 0),
                // Obtain robot speed from drive simulation
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                // Obtain robot facing from drive simulation
                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                // The height at which the coral is ejected
                Meters.of(1.1),
                // The initial speed of the coral
                MetersPerSecond.of(2),
                // The coral is ejected at a 35-degree slope
                Degrees.of(35)));
            }
            // Score Level L4 Code
            if (level == ScoreLevel.L4) {
                SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                    // Obtain robot position from drive simulation
                    driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                    // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                    new Translation2d(-1.15, 0.1),
                    // Obtain robot speed from drive simulation
                    driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    // Obtain robot facing from drive simulation
                    driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                    // The height at which the coral is ejected
                    Meters.of(2.1),
                    // The initial speed of the coral
                    MetersPerSecond.of(1),
                    // The coral is ejected vertically downwards
                    Degrees.of(90)));
            }
        }
    }
}

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToBarge extends SequentialCommandGroup {

    
    public ArmJoint shoulder;
    public ArmJoint elbow;
    public Elevator elevator;
    public Wrist wrist;
    public CoralEndEffector fingeys;

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/BargeScoreCommand/shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("Positions/BargeScoreCommand/shoulder/MidPointDegrees", 0)),
        SafeToSwingElbow(new LoggedTunableNumber("Positions/BargeScoreCommand/shoulder/SafeToSwingElbowDegrees", 40)),
        Final(new LoggedTunableNumber("Positions/BargeScoreCommand/shoulder/FinalDegrees", -85)),
        SafeToMoveElevator(new LoggedTunableNumber("Positions/BargeScoreCommand/shoulder/SafeToMoveElevatorDegrees", -70));

        DoubleSupplier position;
        MutAngle distance;

        ShoulderPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    private enum ElbowPositions {
        Final(new LoggedTunableNumber("Positions/BargeScoreCommand/elbow/FinalDegrees", 65)),
        SafeToMoveElevator(new LoggedTunableNumber("Positions/BargeScoreCommand/elbow/SafeToMoveElevatorDegrees", 50));


        DoubleSupplier position;
        MutAngle distance;

        ElbowPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    private enum WristPositions {
        Starting(new LoggedTunableNumber("Positions/BargeScoreCommand/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/BargeScoreCommand/wrist/FinalDegrees", 0));

        DoubleSupplier position;
        MutAngle distance;

        WristPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    private enum ElevatorPositions {
        Starting(new LoggedTunableNumber("Positions/BargeScoreCommand/elevator/StartingInches", 0)),
        SafeToSwingShoulder(new LoggedTunableNumber("Positions/BargeScoreCommand/elevator/SafeToSwingShoulderInches", 5.0)),
        Final(new LoggedTunableNumber("Positions/BargeScoreCommand/elevator/FinalInches", 26.0)),
        Launch(new LoggedTunableNumber("Positions/BargeScoreCommand/elevator/FinalInches", 20.0)),
        Tolerance(new LoggedTunableNumber("Positions/BargeScoreCommand/elevator/ToleranceInches", 1.0));

        DoubleSupplier position;
        MutDistance distance;

        ElevatorPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Inches.mutable(0.0);
        }

        public Distance distance() {
            this.distance.mut_replace(this.position.getAsDouble(), Inches);
            return this.distance;
        }
    }

    public StowToBarge(ArmJoint shoulder, ArmJoint elbow, Wrist wrist) {
        super( 
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToMoveElevator.position)),
            new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.SafeToMoveElevator.position))
        );
        addRequirements(shoulder, elbow, wrist);
    }
}

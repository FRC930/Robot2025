package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

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

public class BargeScore extends SequentialCommandGroup {
    public ArmJoint shoulder;
    public ArmJoint elbow;
    public Elevator elevator;
    public Wrist wrist;
    public CoralEndEffector fingeys;

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("BargeScoreCommand/shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("BargeScoreCommand/shoulder/MidPointDegrees", 0)),
        SafeToSwingElbow(new LoggedTunableNumber("BargeScoreCommand/shoulder/SafeToSwingElbowDegrees", 40)),
        Final(new LoggedTunableNumber("BargeScoreCommand/shoulder/FinalDegrees", -80));

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
        Starting(new LoggedTunableNumber("BargeScoreCommand/elbow/StartingDegrees", 0)),
        ShoulderSafeSwing(new LoggedTunableNumber("BargeScoreCommand/elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("BargeScoreCommand/elbow/FinalDegrees", 65));

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
        Starting(new LoggedTunableNumber("BargeScoreCommand/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("BargeScoreCommand/wrist/FinalDegrees", 0));

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
        Starting(new LoggedTunableNumber("BargeScoreCommand/elevator/StartingInches", 0)),
        Final(new LoggedTunableNumber("BargeScoreCommand/elevator/FinalInches", 26.0));

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

    public BargeScore(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        super( 
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.position)
                .raceWith(
                    new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.position))
                        .andThen(
                            elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
                                .raceWith(new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.position))
                        )
                    )
                ),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
                .alongWith(elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position))  
        );
        addRequirements(shoulder, elbow, wrist, elevator);
    }
}

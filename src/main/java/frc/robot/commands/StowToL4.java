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

public class StowToL4 extends SequentialCommandGroup {
    public ArmJoint shoulder;
    public ArmJoint elbow;
    public Elevator elevator;
    public Wrist wrist;
    public CoralEndEffector fingeys;

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("MoveToL4Command/shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("MoveToL4Command/shoulder/MidPointDegrees", 110)),
        SafeToSwingElbow(new LoggedTunableNumber("MoveToL4Command/shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("MoveToL4Command/shoulder/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("MoveToL4Command/elbow/StartingDegrees", 0)),
        ShoulderSafeSwing(new LoggedTunableNumber("MoveToL4Command/elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("MoveToL4Command/elbow/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("MoveToL4Command/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("MoveToL4Command/wrist/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("MoveToL4Command/elevator/StartingInches", 0)),
        Final(new LoggedTunableNumber("MoveToL4Command/elevator/FinalInches", 6));

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

    public StowToL4(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector fingeys) {
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
                .alongWith(elevator.getNewSetDistanceCommand(ElevatorPositions.Final.distance().in(Inches)))  
        );
        addRequirements(shoulder, elbow, wrist, elevator);
    }
}

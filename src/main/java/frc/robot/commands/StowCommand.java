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
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowCommand extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("StowCommand/shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("StowCommand/shoulder/MidPointDegrees", 110)),
        SafeToSwingElbow(new LoggedTunableNumber("StowCommand/shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("StowCommand/shoulder/FinalDegrees", 68));

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
        Starting(new LoggedTunableNumber("StowCommand/elbow/StartingDegrees", 0)),
        ShoulderSafeSwing(new LoggedTunableNumber("StowCommand/elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("StowCommand/elbow/FinalDegrees", 65));

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
        Starting(new LoggedTunableNumber("StowCommand/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowCommand/wrist/FinalDegrees", 0));

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
        Starting(new LoggedTunableNumber("StowCommand/elevator/StartingInches", 0)),
        Final(new LoggedTunableNumber("StowCommand/elevator/FinalInches", 0));

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

    public StowCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE, AlgaeEndEffector algaeEE) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.angle().in(Degrees)),
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.angle().in(Degrees))
                .raceWith(
                    new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.angle().in(Degrees)))
                )
                .andThen(
                    elbow.getNewSetAngleCommand(ElbowPositions.Final.angle().in(Degrees))
                        .raceWith(new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.angle().in(Degrees)))                    
                    )
                ),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.angle().in(Degrees))
                .alongWith(elevator.getNewSetDistanceCommand(ElevatorPositions.Final.distance().in(Inches))),
            coralEE.getNewSetVoltsCommand(0)
                .alongWith(algaeEE.getNewSetVoltsCommand(0))
        );
        addRequirements(shoulder, elbow, wrist, elevator, coralEE, algaeEE);
    }
}

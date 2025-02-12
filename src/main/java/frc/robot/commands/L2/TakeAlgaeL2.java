package frc.robot.commands.L2;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.toesies.Toesies;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class TakeAlgaeL2 extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("TakeAlgaeL2/shoulder/StartingDegrees", 10)),
        // MidPoint(new LoggedTunableNumber("StowToL3Command/shoulder/MidPointDegrees", 110)),
        // SafeToSwingElbow(new LoggedTunableNumber("StowToL3Command/shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("TakeAlgaeL2/shoulder/FinalDegrees", 55));

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
        Starting(new LoggedTunableNumber("TakeAlgaeL2/elbow/StartingDegrees", 10)),
        // ShoulderSafeSwing(new LoggedTunableNumber("StowToL3Command/elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("TakeAlgaeL2/elbow/FinalDegrees", 130));

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
        Starting(new LoggedTunableNumber("TakeAlgaeL2/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("TakeAlgaeL2/wrist/FinalDegrees", 0));

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

    public TakeAlgaeL2(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, Toesies toesies, Elevator elevator) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.angle().in(Degrees))
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.angle().in(Degrees))),
            toesies.getNewSetVoltsCommand(6)
            .alongWith(shoulder.getNewSetAngleCommand(ShoulderPositions.Final.angle().in(Degrees)))
            .alongWith(elevator.getNewSetDistanceCommand(16))

            // LOGIC NEEDED FOR INTAKE TO STOW
            // .alongWith(
            //     new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.angle().in(Degrees)))
            //         .andThen(
            //             elbow.getNewSetAngleCommand(ElbowPositions.Final.angle().in(Degrees))
            //                 .alongWith(new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.angle().in(Degrees)))
            //         )
            //     )
            // ),
            // shoulder.getNewSetAngleCommand(ShoulderPositions.Final.angle().in(Degrees))
        );
        addRequirements(shoulder, elbow, wrist, toesies, elevator);
    }
}

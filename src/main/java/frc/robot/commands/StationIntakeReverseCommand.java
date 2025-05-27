package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StationIntakeReverseCommand extends SequentialCommandGroup {

    private static final String className = StationIntakeReverseCommand.class.getSimpleName();

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/" + className + "/shoulder/StartingDegrees", 10)),
        // MidPoint(new LoggedTunableNumber("StowToL3Command/shoulder/MidPointDegrees", 110)),
        // SafeToSwingElbow(new LoggedTunableNumber("StowToL3Command/shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("Positions/" + className + "/shoulder/FinalDegrees", 55));

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
        Starting(new LoggedTunableNumber("Positions/" + className + "/elbow/StartingDegrees", 10)),
        // ShoulderSafeSwing(new LoggedTunableNumber("StowToL3Command/elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("Positions/" + className + "/elbow/FinalDegrees", 33));

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
        Starting(new LoggedTunableNumber("Positions/" + className + "/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/" + className + "/wrist/FinalDegrees", -90));

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

    public StationIntakeReverseCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEndEffector) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position))
            .alongWith(coralEndEffector.getNewSetVoltsCommand(6))

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
        addRequirements(shoulder, elbow, elevator, wrist);
    }
}

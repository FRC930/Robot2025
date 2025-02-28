package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToL3 extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("StowToL3/shoulder/StartingDegrees", 10)),
        Final(new LoggedTunableNumber("StowToL3/shoulder/FinalDegrees", 42.5)),
        Confirm(new LoggedTunableNumber("StowToL3/shoulder/Confirm", 65));

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
        Starting(new LoggedTunableNumber("StowToL3/elbow/StartingDegrees", 10)),
        Final(new LoggedTunableNumber("StowToL3/elbow/FinalDegrees", 20)),
        Confirm(new LoggedTunableNumber("StowToL3/elbow/ConfirmDegrees", -20));

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
        Starting(new LoggedTunableNumber("StowToL3/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowToL3/wrist/FinalDegrees", 0));

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

    public StowToL3(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, Elevator elevator) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position))
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
        addRequirements(shoulder, elbow, wrist, elevator);
    }

    public static Command getNewScoreCommand(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector) {
        return(elbow.getNewSetAngleCommand(-30)
        .alongWith(shoulder.getNewSetAngleCommand(105.0))
        .alongWith(wrist.getNewApplyCoastModeCommand())
        .alongWith(
            new WaitCommand(0.25))
            .andThen(coralEndEffector.getNewSetVoltsCommand(-4)))
        // .andThen(new WaitCommand(0.25))
        .andThen(shoulder.getNewSetAngleCommand(ShoulderPositions.Confirm.position))
        .andThen(new WaitCommand(0.25));
    }

    public static Command getNewStopScoreCommand(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector) {
        return(coralEndEffector.getNewSetVoltsCommand(1)
        .alongWith(shoulder.getNewSetAngleCommand(55))
        .alongWith(elbow.getNewSetAngleCommand(50))
        .alongWith(new WaitCommand(0.2)).andThen(wrist.getNewWristTurnCommand(0)));
    }

}

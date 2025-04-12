package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToL2 extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/StowToL2/shoulder/StartingDegrees", 10)),
        Final(new LoggedTunableNumber("Positions/StowToL2/shoulder/FinalDegrees", 35)),
        Confirm(new LoggedTunableNumber("Positions/StowToL2/shoulder/Confirm", 70));

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
        Starting(new LoggedTunableNumber("Positions/StowToL2/elbow/StartingDegrees", 10)),
        Final(new LoggedTunableNumber("Positions/StowToL2/elbow/FinalDegrees", 20)), 
        Confirm(new LoggedTunableNumber("Positions/StowToL2/elbow/ConfirmDegrees", -20));

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
        Starting(new LoggedTunableNumber("Positions/StowToL2/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowToL2/wrist/FinalDegrees", 0));

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

    public StowToL2(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position))

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

    public StowToL2(ArmJoint shoulder, Elevator elevator, ArmJoint elbow, Wrist wrist,
            CoralEndEffector coralEndEffector) {
        //TODO Auto-generated constructor stub
    }

    public static Command getNewScoreCommand(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector) {
        return elbow.getNewSetAngleCommand(ElbowPositions.Confirm.position)
            .alongWith(wrist.getNewApplyCoastModeCommand())
            .alongWith(new WaitUntilCommand(elbow.getNewAtAngleTrigger(ElbowPositions.Confirm.angle(), Degrees.of(7.0))).withTimeout(1.0))
            .andThen(coralEndEffector.getNewSetVoltsCommand(-4))
            .andThen(shoulder.getNewSetAngleCommand(ShoulderPositions.Confirm.position))
            .andThen(new WaitCommand(0.25));
    }

    public static Command getNewStopScoreCommand(ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector) {
        return(elbow.getNewSetAngleCommand(ElbowPositions.Final.position))
        .alongWith(
            new WaitCommand(0.2)
            .andThen(wrist.getNewWristTurnCommand(0))
        )
        .andThen(coralEndEffector.getNewSetVoltsCommand(1));
    }

    public static Trigger getNewAtScoreTrigger(ArmJoint shoulder, ArmJoint elbow, Wrist wrist) {
        return shoulder.getNewAtAngleTrigger(Degrees.of(ShoulderPositions.Final.position.getAsDouble()), Degrees.of(20.0))
            .and(elbow.getNewAtAngleTrigger(Degrees.of(ElbowPositions.Final.position.getAsDouble()), Degrees.of(5.0)))
            .and(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Final.position.getAsDouble()), Degrees.of(5.0)));
    }

}

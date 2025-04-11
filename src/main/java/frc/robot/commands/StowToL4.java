package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToL4 extends SequentialCommandGroup {

    

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/StowToL4Command/shoulder/StartingDegrees", 97.5)),
        SafeToMoveElevator(new LoggedTunableNumber("Positions/StowToL4Command/shoulder/SafeToSwingElbowDegrees", 60)),
        Final(new LoggedTunableNumber("Positions/StowToL4Command/shoulder/FinalDegrees", -60));

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
        Starting(new LoggedTunableNumber("Positions/StowToL4Command/elbow/StartingDegrees", 65)),
        MidPoint(new LoggedTunableNumber("Positions/StowToL4Command/elbow/ShoulderSafeSwingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowToL4Command/elbow/FinalDegrees", -100)),
        Confirm(new LoggedTunableNumber("Positions/StowToL4Command/elbow/ConfirmDegrees", -140));

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
        Starting(new LoggedTunableNumber("Positions/StowToL4Command/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowToL4Command/wrist/FinalDegrees", 0));

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
        Starting(new LoggedTunableNumber("Positions/StowToL4Command/elevator/StartingInches", 0)),
        SafeToSwingShoulder(new LoggedTunableNumber("Positions/StowToL4Command/elevator/SafeToSwingShoulderInches", 5.0)),
        Final(new LoggedTunableNumber("Positions/StowToL4Command/elevator/FinalInches", 18));

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

    /**
     * Regular StowToL4 using both Elevator and Arm
     */
    public StowToL4(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        super(
            new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.Starting.position)),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToMoveElevator.position)),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.MidPoint.position)
                .andThen(
                    new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToMoveElevator.position))
                        .andThen(
                            elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
                        )
                        .andThen(
                            new WaitUntilCommand(elbow.getNewLessThanAngleTrigger(ElbowPositions.Final.position.getAsDouble() + 5.0))
                        )
                )
        );
        addRequirements(shoulder, elbow, wrist, elevator);
    }

    /**
     * {@summary StowToL4 with only Elevator motions}
     * Typically used in autos, when the arm is already in position but the elevator isn't because Center Of Mass!
     * @param shoulder Is only used for checking safezones
     */
    public StowToL4(Elevator elevator, ArmJoint shoulder) {
        super(
            new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.Starting.position)),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position)
        );
        addRequirements(elevator);
    }

    /**
     * {@summary StowToL4 without Elevator motions, only arm motions}
     * Typically used in autos, when we want to move while arm is ready for L4 because efficiency
     */
    public StowToL4(ArmJoint shoulder, ArmJoint elbow, Wrist wrist) {
        super(
            new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.Starting.position)),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleWithSlotCommand(ShoulderPositions.Final.position, 1),
            elbow.getNewSetAngleCommand(ElbowPositions.MidPoint.position)
                .andThen(
                    new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToMoveElevator.position))
                        .andThen(
                            elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
                        )
                        .andThen(
                            new WaitUntilCommand(elbow.getNewLessThanAngleTrigger(ElbowPositions.Final.position.getAsDouble() + 5.0))
                        )
                )
        );
        addRequirements(shoulder, elbow, wrist);
    }

    public static Trigger getNewAtScoreTrigger(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        return shoulder.getNewAtAngleTrigger(Degrees.of(ShoulderPositions.Final.position.getAsDouble()), Degrees.of(20.0))
            .and(elbow.getNewAtAngleTrigger(Degrees.of(ElbowPositions.Final.position.getAsDouble()), Degrees.of(5.0)))
            .and(elevator.getNewAtDistanceTrigger(Inches.of(ElevatorPositions.Final.position.getAsDouble()), Inches.of(2.0)))
            .and(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Final.position.getAsDouble()), Degrees.of(5.0)));
    }

    public static Trigger getNewArmAtL4Trigger(ArmJoint shoulder, ArmJoint elbow, Wrist wrist) {
        return shoulder.getNewAtAngleTrigger(Degrees.of(ShoulderPositions.Final.position.getAsDouble()), Degrees.of(6.0))
            .and(elbow.getNewAtAngleTrigger(Degrees.of(ElbowPositions.Final.position.getAsDouble()), Degrees.of(5.0)))
            .and(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Final.position.getAsDouble()), Degrees.of(5.0)));
    }

    public static Command getNewScoreCommand(ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector) {
        return(
            elbow.getNewSetAngleCommand(ElbowPositions.Confirm.position)
            .alongWith(wrist.getNewApplyCoastModeCommand())
            .alongWith(new WaitUntilCommand(elbow.getNewAtAngleTrigger(ElbowPositions.Confirm.angle(), Degrees.of(5.0))).withTimeout(1.0))
        )
        .andThen(coralEndEffector.getNewSetVoltsCommand(-4));
    }

    public static Command getNewStopScoreCommand(ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector, Drive drive) {
        return wrist.getNewWristTurnCommand(0)
                .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position))
            .andThen(coralEndEffector.getNewSetVoltsCommand(-2.0));
    }

}

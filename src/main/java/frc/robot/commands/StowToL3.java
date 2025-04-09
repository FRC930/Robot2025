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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToL3 extends SequentialCommandGroup {

    

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/StowToL3/shoulder/StartingDegrees", 97.5)),
        Final(new LoggedTunableNumber("Positions/StowToL3/shoulder/FinalDegrees", 32.5)),
        Confirm(new LoggedTunableNumber("Positions/StowToL3/shoulder/Confirm", 47.0));

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
        Starting(new LoggedTunableNumber("Positions/StowToL3/elbow/StartingDegrees", 10)),
        Final(new LoggedTunableNumber("Positions/StowToL3/elbow/FinalDegrees", 20)),
        Confirm(new LoggedTunableNumber("Positions/StowToL3/elbow/ConfirmDegrees", -50));

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
        Starting(new LoggedTunableNumber("Positions/StowToL3/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowToL3/wrist/FinalDegrees", 0));

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
        Starting(new LoggedTunableNumber("Positions/StowToL3Command/elevator/StartingInches", 0)),
        SafeToSwingShoulder(new LoggedTunableNumber("Positions/StowToL3Command/elevator/SafeToSwingShoulderInches", 5.0)),
        Final(new LoggedTunableNumber("Positions/StowToL3Command/elevator/FinalInches", 15));

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

    public StowToL3(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, Elevator elevator) {
        super(
            new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.Starting.position)),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position))
            .alongWith(elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position))
        );
        addRequirements(shoulder, elbow, wrist, elevator);
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
        return new WaitCommand(0.2)
            .andThen(coralEndEffector.getNewSetVoltsCommand(1));
    }

    public static Trigger getNewAtScoreTrigger(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist) {
        return shoulder.getNewAtAngleTrigger(Degrees.of(ShoulderPositions.Final.position.getAsDouble()), Degrees.of(10.0))
            .and(elbow.getNewAtAngleTrigger(Degrees.of(ElbowPositions.Final.position.getAsDouble()), Degrees.of(5.0)))
            .and(elevator.getNewAtDistanceTrigger(Inches.of(ElevatorPositions.Final.position.getAsDouble()), Inches.of(2.0)))
            .and(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Final.position.getAsDouble()), Degrees.of(5.0)));
    }

}

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
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class TakeAlgaeL3 extends SequentialCommandGroup {

    

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/TakeAlgaeL3/shoulder/StartingDegrees", 10)),
        // MidPoint(new LoggedTunableNumber("StowToL3Command/shoulder/MidPointDegrees", 110)),
        // SafeToSwingElbow(new LoggedTunableNumber("StowToL3Command/shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("Positions/TakeAlgaeL3/shoulder/FinalDegrees", -35));

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
        Starting(new LoggedTunableNumber("Positions/TakeAlgaeL3/elbow/StartingDegrees", 10)),
        // ShoulderSafeSwing(new LoggedTunableNumber("StowToL3Command/elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("Positions/TakeAlgaeL3/elbow/FinalDegrees", 57));

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
        Starting(new LoggedTunableNumber("Positions/TakeAlgaeL3/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/TakeAlgaeL3/wrist/FinalDegrees", 0));

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
        Starting(new LoggedTunableNumber("Positions/TakeAlgaeL3/elevator/StartingInches", 0)),
        Final(new LoggedTunableNumber("Positions/TakeAlgaeL3/elevator/FinalInches", 7));

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

    public TakeAlgaeL3(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, AlgaeEndEffector algaeEE, Elevator elevator) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position)
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position)),
            algaeEE.getNewSetVoltsCommand(8)
            .alongWith(shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position))
            .alongWith(elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position)),
            new WaitUntilCommand(shoulder.getNewAtAngleTrigger(ShoulderPositions.Final.position, ()->(3))).withTimeout(0.5)

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
        addRequirements(shoulder, elbow, wrist, algaeEE, elevator);
    }
}

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

public class StowToAlgaeStow extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("StowToAlgaeStow/shoulder/StartingDegrees", 10)),
        // MidPoint(new LoggedTunableNumber("StowToL3Command/shoulder/MidPointDegrees", 110)),
        // SafeToSwingElbow(new LoggedTunableNumber("StowToL3Command/shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("StowToAlgaeStow/shoulder/FinalDegrees", 35)), 
        SafeToSwingElbow(new LoggedTunableNumber("StowToAlgaeStow/shoulder/SafeToSwingElbow", 35)), 
        MidPoint(new LoggedTunableNumber("StowToAlgaeStow/shoulder/MidPoint", 35));

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
        Starting(new LoggedTunableNumber("StowToAlgaeStow/elbow/StartingDegrees", 10)),
        // ShoulderSafeSwing(new LoggedTunableNumber("StowToL3Command/elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("StowToAlgaeStow/elbow/FinalDegrees", -160)), 
        ShoulderSafeSwing(new LoggedTunableNumber("StowToAlgaeStow/elbow/lmao", -160));

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

    private enum ElevatorPositions {
        Starting(new LoggedTunableNumber("StowToAlgaeStow/elevator/StartingInches", 0)),
        Final(new LoggedTunableNumber("StowToAlgaeStow/elevator/FinalInches", 0));

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

    private enum WristPositions {
        Starting(new LoggedTunableNumber("StowToAlgaeStow/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowToAlgaeStow/wrist/FinalDegrees", 0));

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

    public StowToAlgaeStow(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, AlgaeEndEffector algaeEE) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.position)
                .raceWith(
                    new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.position))
                )
                .andThen(
                    elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
                        .raceWith(new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.position))                    
                    )
                ),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
                .alongWith(elevator.getNewSetDistanceCommand(ElevatorPositions.Final.distance().in(Inches))),
            algaeEE.getNewSetVoltsCommand(0)

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
        addRequirements(shoulder, elbow,  elevator, wrist, algaeEE);
    }
}

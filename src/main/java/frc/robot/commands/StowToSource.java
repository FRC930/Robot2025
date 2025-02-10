package frc.robot.commands;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;


public class StowToSource extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("StowToSourceCommand/Shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("StowToSourceCommand/Shoulder/MidPointDegrees", 110)),
        SafeToSwingElbow(new LoggedTunableNumber("StowToSourceCommand/Shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("StowToSourceCommand/Shoulder/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("StowToSourceCommand/Elbow/StartingDegrees", 0)),
        ShoulderSafeSwing(new LoggedTunableNumber("StowToSourceCommand/Elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("StowToSourceCommand/Elbow/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("StowToSourceCommand/Wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowToSourceCommand/Wrist/FinalDegrees", 90));

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

    public StowToSource(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, Fingeys fingeys) {
                super(
            wrist.getNewWristTurnCommand(WristPositions.Final.angle().in(Degrees)),
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.angle().in(Degrees))
                .alongWith(
                    new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.angle().in(Degrees)))
                        .andThen(
                            elbow.getNewSetAngleCommand(ElbowPositions.Final.angle().in(Degrees))
                                .alongWith(new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.angle().in(Degrees)))
                        )
                    )
                ),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.angle().in(Degrees))
        );
        addRequirements(shoulder, elbow, wrist, fingeys);
    }
}
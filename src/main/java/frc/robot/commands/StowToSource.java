package frc.robot.commands;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;


public class StowToSource extends SequentialCommandGroup {

    

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/StowToSourceCommand/Shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("Positions/StowToSourceCommand/Shoulder/MidPointDegrees", 110)),
        SafeToSwingElbow(new LoggedTunableNumber("Positions/StowToSourceCommand/Shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("Positions/StowToSourceCommand/Shoulder/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("Positions/StowToSourceCommand/Elbow/StartingDegrees", 0)),
        ShoulderSafeSwing(new LoggedTunableNumber("Positions/StowToSourceCommand/Elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("Positions/StowToSourceCommand/Elbow/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("Positions/StowToSourceCommand/Wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowToSourceCommand/Wrist/FinalDegrees", 90));

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

    public StowToSource(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector fingeys) {
                super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.position)
                .alongWith(
                    new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.position))
                        .andThen(
                            elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
                                .alongWith(new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.position))
                        )
                    )
                ),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
        );
        addRequirements(shoulder, elbow, wrist, fingeys);
    }
}
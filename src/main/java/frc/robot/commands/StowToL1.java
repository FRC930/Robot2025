package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToL1 extends SequentialCommandGroup {

    

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/StowToL1/shoulder/StartingDegrees", 10)),
        SafeToTurnWrist(new LoggedTunableNumber("Positions/StowToL1/shoulder/SafeToTurnWristDegrees", 80)),
        Final(new LoggedTunableNumber("Positions/StowToL1/shoulder/FinalDegrees", 55));

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
        Starting(new LoggedTunableNumber("Positions/StowToL1/elbow/StartingDegrees", 10)),
        SafeToTurnWrist(new LoggedTunableNumber("Positions/StowToL1/elbow/SafeToTurnWristDegrees", 50)),
        Final(new LoggedTunableNumber("Positions/StowToL1/elbow/FinalDegrees", -20));

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
        Starting(new LoggedTunableNumber("Positions/StowToL1/wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowToL1/wrist/FinalDegrees", 90));

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

    public StowToL1(ArmJoint shoulder, ArmJoint elbow, Wrist wrist) {
        super(
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position)),  
            new WaitUntilCommand(elbow.getNewLessThanAngleTrigger(ElbowPositions.SafeToTurnWrist.position)
                .and(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToTurnWrist.position))).withTimeout(0.2),
            wrist.getNewWristTurnCommand(WristPositions.Final.position)
        );
    }
    public static Command getNewScoreCommand(CoralEndEffector coralEndEffector) {
        return(coralEndEffector.getNewSetVoltsCommand(-2))
        .andThen(new WaitUntilCommand(coralEndEffector.hasCoralTrigger().negate()).withTimeout(0.2));
    }
    public static Command getNewStopScoreCommand(CoralEndEffector coralEndEffector){
        return(coralEndEffector.getNewSetVoltsCommand(0));
    }

    public static Trigger getNewAtScoreTrigger(ArmJoint shoulder, ArmJoint elbow, Wrist wrist) {
        return shoulder.getNewAtAngleTrigger(Degrees.of(ShoulderPositions.Final.position.getAsDouble()), Degrees.of(20.0))
            .and(elbow.getNewAtAngleTrigger(Degrees.of(ElbowPositions.Final.position.getAsDouble()), Degrees.of(10.0)))
            .and(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Final.position.getAsDouble()), Degrees.of(5.0)));
    }
}

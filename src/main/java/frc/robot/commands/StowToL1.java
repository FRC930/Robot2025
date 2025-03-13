package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StowToL1 extends SequentialCommandGroup {
    private enum ShoulderPositions {
        SafeToTurnWrist(new LoggedTunableNumber("StowToL1/shoulder/SafeToTurnWristDegrees", 90)),
        Final(new LoggedTunableNumber("StowToL1/shoulder/FinalDegrees", 60));

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
        SafeToTurnWrist(new LoggedTunableNumber("StowToL1/elbow/SafeToTurnWristDegrees", 90)),
        Final(new LoggedTunableNumber("StowToL1/elbow/FinalDegrees", -25));

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
        Final(new LoggedTunableNumber("StowToL1/wrist/FinalDegrees", 90));

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
                .and(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToTurnWrist.position))),
            wrist.getNewWristTurnCommand(WristPositions.Final.position)
        );
    }
    public static Command getNewScoreCommand(CoralEndEffector coralEndEffector) {
        return(coralEndEffector.getNewSetVoltsCommand(-4));
    }
    public static Command getNewStopScoreCommand(CoralEndEffector coralEndEffector){
        return(coralEndEffector.getNewSetVoltsCommand(0));
    }
}

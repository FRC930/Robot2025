package frc.robot.commands.L2;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class L2Outtake extends SequentialCommandGroup {
    
    private enum ShoulderPositions {
        Ready(new LoggedTunableNumber("L2Outtake/shoulder/ReadyDegrees",55)),
        Final(new LoggedTunableNumber("L2Outtake/shoulder/FinalDegrees", 55));

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
        Starting(new LoggedTunableNumber("L2Outtake/elbow/FinalDegrees",50)),
        Final(new LoggedTunableNumber("L2Outtake/elbow/FinalDegrees", 130));

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
        Final(new LoggedTunableNumber("L2Outtake/wrist/FinalDegrees", 0));

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

    public L2Outtake(
        ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector fingeys) {
        super(
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
            .alongWith(shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position))
                .alongWith(new WaitCommand(0.5))
            .andThen(fingeys.getNewSetVoltsCommand(-4))
        );
        addRequirements(elbow, wrist, fingeys);
    }

    public static Command getResetCommand(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector fingeys) {
        return new SequentialCommandGroup(
            elbow.getNewSetAngleCommand(ElbowPositions.Starting.position)
            .alongWith(shoulder.getNewSetAngleCommand(ShoulderPositions.Ready.position))
            .alongWith(fingeys.getNewSetVoltsCommand(1))
        );
    }
}

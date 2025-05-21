package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.IntakeIOSim;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class StationIntakeCommand extends SequentialCommandGroup {

    private static final String className = StationIntakeCommand.class.getSimpleName();

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/" + className + "/shoulder/StartingDegrees", -45.0)),
        Final(new LoggedTunableNumber("Positions/" + className + "/shoulder/FinalDegrees", 180.0-58.5));

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
        Starting(new LoggedTunableNumber("Positions/" + className + "/elbow/StartingDegrees", 50.0)),
        Final(new LoggedTunableNumber("Positions/" + className + "/elbow/FinalDegrees", 142.5));

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
        Starting(new LoggedTunableNumber("Positions/" + className + "/wrist/StartingDegrees", 0.0)),
        Final(new LoggedTunableNumber("Positions/" + className + "/wrist/FinalDegrees", -90.0));

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
        Starting(new LoggedTunableNumber("Positions/" + className + "/elevator/StartingInches", 2.5));

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

    public StationIntakeCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEndEffector, IntakeIOSim intakeSim) {
        super(
            new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.Starting.position)),
            new WaitUntilCommand(elevator.getNewLessThanDistanceTrigger(ElevatorPositions.Starting.position)),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
            .alongWith(elbow.getNewSetAngleCommand(ElbowPositions.Final.position))
            .alongWith(coralEndEffector.getNewSetVoltsCommand(6.0))
            .alongWith(intakeSim.setRunnningCommand(true))
            .alongWith(intakeSim.dropCoralCommand())
        );
        addRequirements(shoulder, elbow, elevator, wrist);
    }
}

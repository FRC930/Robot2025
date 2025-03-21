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
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class OverhandIntakeToL4 extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Final(new LoggedTunableNumber("OverhandIntakeToL4/shoulder/FinalDegrees", -55.0));

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
        Final(new LoggedTunableNumber("OverhandIntakeToL4/elbow/FinalDegrees", -100.0));

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
        Final(new LoggedTunableNumber("OverhandIntakeToL4/wrist/FinalDegrees", 0.0));

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
        Final(new LoggedTunableNumber("OverhandIntakeToL4/elevator/FinalInches", 16.0));

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
    public OverhandIntakeToL4(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, CoralEndEffector coralEE) {
        super(
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position),
            coralEE.getNewSetVoltsCommand(1.0)
        );
        addRequirements(shoulder, elbow, wrist, elevator, coralEE);
    }

    /**
     * No elevator used
     * 
     * @param shoulder
     * @param elbow
     * @param wrist
     * @param coralEE
     */
    public OverhandIntakeToL4(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector coralEE) {
        super(
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            coralEE.getNewSetVoltsCommand(1.0)
        );
        addRequirements(shoulder, elbow, wrist, coralEE);
    }

    /**
     * No elevator used
     * 
     * @param shoulder
     * @param elbow
     * @param wrist
     * @param coralEE
     */
    public OverhandIntakeToL4(Elevator elevator) {
        super(
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position)
        );
        addRequirements(elevator);
    }

}

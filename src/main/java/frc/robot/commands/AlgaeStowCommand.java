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

public class AlgaeStowCommand extends SequentialCommandGroup {

    private enum ShoulderPositions {
        // Sets the degrees that the elevator can go to when needed
        Starting(new LoggedTunableNumber("StowToAlgaeStow/shoulder/StartingDegrees", 0.0)),
        Final(new LoggedTunableNumber("StowToAlgaeStow/shoulder/FinalDegrees", 20.0)), 
        SafeToLowerElevator(new LoggedTunableNumber("StowToAlgaeStow/shoulder/SafeToLowerElevator", -45.0));

        DoubleSupplier position;
        MutAngle distance;

        //sets position to be position receved
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
        // Set elbow position possibilies
        Starting(new LoggedTunableNumber("StowToAlgaeStow/elbow/StartingDegrees", 0.0)),
        ShoulderSafeSwing(new LoggedTunableNumber("StowToL3Command/elbow/ShoulderSafeSwingDegrees", 70.0)),
        Final(new LoggedTunableNumber("StowToAlgaeStow/elbow/FinalDegrees", 70.0));

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
        // Set elevator position possibilies
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

    public AlgaeStowCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, AlgaeEndEffector algaeEE) {
        super(
            //Sets the angles to what they were defined as
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            new WaitUntilCommand(shoulder.getNewGreaterThanAngleTrigger(ShoulderPositions.SafeToLowerElevator.position)),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.distance().in(Inches)),
            algaeEE.getNewSetVoltsCommand(4.0)
        );
        addRequirements(shoulder, elbow,  elevator, wrist, algaeEE);
    }
}

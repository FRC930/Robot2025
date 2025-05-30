package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class AlgaeStowCommand extends SequentialCommandGroup {

    /*
     * A container for the shoulder positions being sent to the command
     */
    private enum ShoulderPositions {
        Final(new LoggedTunableNumber("Positions/AlgaeStowCommand/shoulder/FinalDegrees", 60.0));

        DoubleSupplier position;

        ShoulderPositions(DoubleSupplier position) {
            this.position = position;
        }
    }

    /*
     * A container for elbow positions being sent to the command
     */
    private enum ElbowPositions {
        Final(new LoggedTunableNumber("Positions/AlgaeStowCommand/elbow/FinalDegrees", 155));

        DoubleSupplier position;

        ElbowPositions(DoubleSupplier position) {
            this.position = position;
        } 
    }

    /*
     * A container for the elevator positions being sent to the command
     */
    private enum ElevatorPositions {
        Final(new LoggedTunableNumber("Positions/AlgaeStowCommand/elevator/FinalInches", 0));

        DoubleSupplier position;

        ElevatorPositions(DoubleSupplier position) {
            this.position = position;
        }
    }

    /*
     * Container for the wrist positions being sent to the command
     */
    private enum WristPositions {
        Final(new LoggedTunableNumber("Positions/AlgaeStowCommand/wrist/FinalDegrees", 0));

        DoubleSupplier position;

        WristPositions(DoubleSupplier position) {
            this.position = position;
        }
    }

    //Instant command that sets robot joints to the algae stow position
    public AlgaeStowCommand(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, AlgaeEndEffector algaeEE) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            elevator.getNewSetDistanceCommand(ElevatorPositions.Final.position),
            algaeEE.getNewSetVoltsCommand(2.5)
        );
        addRequirements(shoulder, elbow,  elevator, wrist, algaeEE);
    }
}

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;


public class StowToAlgaeIntake extends SequentialCommandGroup {

    

    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Shoulder/MidPointDegrees", 110)),
        SafeToSwingElbow(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Shoulder/FinalDegrees", 90));
        

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
        Starting(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Elbow/StartingDegrees", 0)),
        ShoulderSafeSwing(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Elbow/FinalDegrees", 90));

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
        Starting(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("Positions/StowToAlgaeIntake/Wrist/FinalDegrees", 180));

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
    
    private enum AlgaeIntakePositions {
        Starting(new LoggedTunableNumber("Positions/StowToAlgaeIntake/AlgaeIntake/StartingVolts", 0)),
        Final(new LoggedTunableNumber("Positions/StowToAlgaeIntake/AlgaeIntake/FinalVolts", 2));

        DoubleSupplier voltage;
        MutVoltage volts;

        AlgaeIntakePositions(DoubleSupplier voltage) {
            this.voltage = voltage;
            this.volts = Volts.mutable(0.0);
        }

        public Voltage voltage() {
            this.volts.mut_replace(this.voltage.getAsDouble(), Volts);
            return this.volts;
        }
    }

    public StowToAlgaeIntake(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector fingeys, AlgaeEndEffector algaeIntake) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.position),
            algaeIntake.getNewSetVoltsCommand(AlgaeIntakePositions.Final.voltage)
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
        addRequirements(shoulder, elbow, wrist, fingeys, algaeIntake);
    }
}
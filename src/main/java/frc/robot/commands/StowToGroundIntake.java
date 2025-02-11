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
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.toesies.Toesies;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;


public class StowToGroundIntake extends SequentialCommandGroup {
    private enum ShoulderPositions {
        Starting(new LoggedTunableNumber("StowToGroundIntake/Shoulder/StartingDegrees", 0)),
        MidPoint(new LoggedTunableNumber("StowToGroundIntake/Shoulder/MidPointDegrees", 110)),
        SafeToSwingElbow(new LoggedTunableNumber("StowToGroundIntake/Shoulder/SafeToSwingElbowDegrees", 100)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Shoulder/FinalDegrees", 95));
        

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
        Starting(new LoggedTunableNumber("StowToGroundIntake/Elbow/StartingDegrees", 0)),
        ShoulderSafeSwing(new LoggedTunableNumber("StowToGroundIntake/Elbow/ShoulderSafeSwingDegrees", 45)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Elbow/FinalDegrees", -90));

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
        Starting(new LoggedTunableNumber("StowToGroundIntake/Wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Wrist/FinalDegrees", 180));

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

    public StowToGroundIntake(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, Fingeys fingeys) {
        super(
            wrist.getNewWristTurnCommand(WristPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.position)
                .alongWith(
                    new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.position))
                )
                .andThen(
                    elbow.getNewSetAngleCommand(ElbowPositions.Final.position)
                    .alongWith(
                        new WaitUntilCommand(elbow.getNewLessThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.position))
                    )
                ),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position)
        );
        addRequirements(shoulder, elbow, wrist, fingeys);
    }
}
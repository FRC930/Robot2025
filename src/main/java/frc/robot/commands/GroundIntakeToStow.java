package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.fingeys.Fingeys;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class GroundIntakeToStow extends SequentialCommandGroup {

    private enum ShoulderPositions {
        Intake(new LoggedTunableNumber("GroundIntakeToStowCommand/shoulder/IntakeDegrees", 10)),
        MidPoint(new LoggedTunableNumber("GroundIntakeToStowCommand/shoulder/MidPointDegrees", 110)),
        SafeToSwingElbow(new LoggedTunableNumber("GroundIntakeToStowCommand/shoulder/SafeToSwingElbowDegrees", 100)),
        Stow(new LoggedTunableNumber("GroundIntakeToStowCommand/shoulder/StowDegrees", 68));

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
        Intake(new LoggedTunableNumber("GroundIntakeToStowCommand/elbow/IntakeDegrees", -95)),
        ShoulderSafeSwing(new LoggedTunableNumber("GroundIntakeToStowCommand/elbow/ShoulderSafeSwingDegrees", 45)),
        Stow(new LoggedTunableNumber("GroundIntakeToStowCommand/elbow/StowDegrees", 65));

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
        Intake(new LoggedTunableNumber("GroundIntakeToStowCommand/wrist/IntakeDegrees", 0)),
        Stow(new LoggedTunableNumber("GroundIntakeToStowCommand/wrist/StowDegrees", 0));

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

    public GroundIntakeToStow(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, Fingeys fingeys) {
        super(
            //STEP 1:
            wrist.getNewWristTurnCommand(WristPositions.Stow.position), // Start wrist turn
            //STEP 2:
            shoulder.getNewSetAngleCommand(ShoulderPositions.MidPoint.position) // Start bringing shoulder up
                .alongWith( // Wait until shoulder is up
                    new WaitUntilCommand(shoulder.getNewLessThanAngleTrigger(ShoulderPositions.SafeToSwingElbow.position))
                )
            .andThen(
                elbow.getNewSetAngleCommand(ElbowPositions.Stow.position) // Start bringing elbow up
                .alongWith( //Wait until elbow is up
                    new WaitUntilCommand(elbow.getNewGreaterThanAngleTrigger(ElbowPositions.ShoulderSafeSwing.position))
                )
            ),
            //STEP 3:
            shoulder.getNewSetAngleCommand(ShoulderPositions.Stow.position) // Start bringing shoulder forward to stow
        );
        addRequirements(shoulder, elbow, wrist, fingeys);
    }
}
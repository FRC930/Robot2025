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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;


public class StowToGroundIntake extends SequentialCommandGroup {
    private static enum ShoulderPositions {
        Starting(new LoggedTunableNumber("StowToGroundIntake/Shoulder/StartingDegrees", 90)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Shoulder/FinalDegrees", 55.5));
        

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

    private static enum ElbowPositions {
        Starting(new LoggedTunableNumber("StowToGroundIntake/Elbow/StartingDegrees",65)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Elbow/FinalDegrees", 158));

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

    private static enum WristPositions {
        Starting(new LoggedTunableNumber("StowToGroundIntake/Wrist/StartingDegrees", 0)),
        Final(new LoggedTunableNumber("StowToGroundIntake/Wrist/FinalDegrees", 90));

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

    private static enum IntakeExtenderPositions {
        Final(new LoggedTunableNumber("StowToGroundIntake/Wrist/FinalDegrees", -144));

        DoubleSupplier position;
        MutAngle distance;

        IntakeExtenderPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }



    /**
     * The initial stow to ground intake command. Prepares the arm to take from the ground intake
     * @param shoulder
     * @param elbow
     * @param wrist
     * @param fingeys
     */
    public StowToGroundIntake(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector fingeys) {
        addRequirements(shoulder, elbow, wrist, fingeys);
        addCommands(
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            wrist.getNewWristTurnCommand(-90)
        );
    }

    /**
     * The second part of the ground intake. Spins up the ground intake.
     * If the ground intake has a coral, moves it to the ready pos.
     * <i> This will run forever until interrupted, so make sure to add a kill to it </i>
     * @return
     */
    public static Command getRunGroundIntakeCommand(Intake intake, IntakeExtender extender) {
        return intake.getNewSetVoltsCommand(6)
                .alongWith(extender.getNewIntakeExtenderTurnCommand(-144))
                .alongWith(new WaitUntilCommand(extender.getNewAtAngleTrigger(Degrees.of(-144), Degrees.of(10))))
                .andThen(new WaitUntilCommand(intake.hasCoralTrigger()))
                .andThen(intake.getNewSetVoltsCommand(0))
                .andThen(extender.getNewIntakeExtenderTurnCommand(0));//TODO -25 this is the VALUE for STOW but currently is not getting there
    }

    /**
     * The third part of the ground intake. Raises the ground intake, takes a coral from it, then moves it to the ready pos
     * @return
     */
    public static Command getTakeCoralFromGroundIntakeCommand(Intake intake, IntakeExtender extender, ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector coralEndEffector) {
        return 
        new WaitUntilCommand(wrist.getNewAtAngleTrigger(Degrees.of(-90), Degrees.of(1)))
        .andThen(coralEndEffector.getNewSetVoltsCommand(6))
            .alongWith(extender.getNewIntakeExtenderTurnCommand(0))
        .andThen(new WaitUntilCommand(extender.getNewAtAngleTrigger(Degrees.of(0), Degrees.of(3))))
        .andThen(intake.getNewSetVoltsCommand(-5))
        .andThen(new WaitUntilCommand(coralEndEffector.hasCoralTrigger())) 
        .andThen(intake.getNewSetVoltsCommand(0))
        .andThen(
            extender.getNewIntakeExtenderTurnCommand(-40) //TODO -25 this is the VALUE for STOW but currently is not getting there
            .alongWith(coralEndEffector.getNewSetVoltsCommand(1))
        );
    }

    /**
     * The final part of the ground intake. Returns the arm (With the coral) to the stow pos
     * @param shoulder
     * @param elbow
     * @param wrist
     * @param fingeys
     * @return
     */
    public static Command getReturnToStowCommand(ArmJoint shoulder, ArmJoint elbow, Wrist wrist, CoralEndEffector fingeys, IntakeExtender extender) {
        return extender.getNewIntakeExtenderTurnCommand(-45)
        .andThen(new WaitUntilCommand(extender.getNewAtAngleTrigger(Degrees.of(-45), Degrees.of(5))))
        .andThen(wrist.getNewWristTurnCommand(WristPositions.Starting.position))
        .andThen(new WaitUntilCommand(wrist.getNewAtAngleTrigger(Degrees.of(WristPositions.Starting.position.getAsDouble()), Degrees.of(1))))
        .andThen(shoulder.getNewSetAngleCommand(ShoulderPositions.Starting.position))
        .andThen(elbow.getNewSetAngleCommand(ElbowPositions.Starting.position));
    }
}
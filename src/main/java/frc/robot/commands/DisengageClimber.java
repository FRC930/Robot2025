package frc.robot.commands;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.LoggedTunableNumber;

public class DisengageClimber extends SequentialCommandGroup {

    
    private static final String className = DisengageClimber.class.getSimpleName();

    private enum ShoulderPositions {
        Final(new LoggedTunableNumber(className + "/shoulder/FinalEndgameDegrees", 155.0));

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
        Final(new LoggedTunableNumber(className + "/elbow/FinalEndgameDegrees", 230.0));

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
    

    
    public DisengageClimber(Climber climber, ArmJoint elbow, ArmJoint shoulder) {
        super(
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            climber.getNewSetServoAngleCommand(0.0),
            new WaitCommand(0.5),
            climber.getNewSetVoltsCommand(10.0)
        );
    }
}
// 158 is disengaged fully
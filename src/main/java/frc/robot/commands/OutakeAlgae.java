package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;

public class OutakeAlgae extends SequentialCommandGroup {
    public OutakeAlgae(AlgaeEndEffector toesies) {
        super(
            //TODO processing outtake is -6, barge is -12
            toesies.getNewSetVoltsCommand(-12)
        );
        addRequirements(toesies);
    }
}
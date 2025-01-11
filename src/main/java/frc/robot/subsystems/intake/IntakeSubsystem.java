package frc.robot.subsystems.intake;

public class IntakeSubsystem {
    private IntakeIO intakeIO;
    public void setSpeed(double speed){
        intakeIO.setSpeed(speed);
    }
}

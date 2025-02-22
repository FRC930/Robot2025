package frc.robot.commands;

import java.lang.ModuleLayer.Controller;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AlignTx extends Command {
    private Drive m_drive;
    private Vision m_limeLight;
    private int m_cameraIndex;
    private Controller co_controller;
    private LinearFilter movingAverageFilter;
    private double m_strafe = 0.0;
    private double m_direction;
    private double slope = 0.9; //Needs to be changed to the current slope
    private double m_distance;  
    private PIDController pid = new PIDController(0.0065, 0.0, 0.0);
    private final double MAX_STRAFE = 0.3; 
    private final double MAX_THROTTLE = 1.0;
    private final double MAX_SPEED = 2.0;


    public AlignTx(Drive drive, Vision limeLight, int cameraIndex) {
        this.m_drive = drive;
        this.m_limeLight = limeLight;
        this.m_cameraIndex = cameraIndex;
        
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_direction = -1.0; //Currently camera in back of robot (go backwards) when camera in front use 1.0
        movingAverageFilter = LinearFilter.movingAverage(3);
        if(co_controller != null) {
           return;
        }
        
        // highPassFilter = LinearFilter.highPass(HIGH_PASS_SECONDS, 0.02);
        
    }

    @Override
    public void execute() {
        Rotation2d tx = m_limeLight.getTargetX(m_cameraIndex); // degrees left and right from crosshair
        Rotation2d ty = m_limeLight.getTargetY(m_cameraIndex); // degrees up and down from crosshair

        double linearTX = movingAverageFilter.calculate(tx.getDegrees() <= -100.0 && ty.getDegrees() <= -100.0? 0.0 : (ty.getDegrees()/slope) - tx.getDegrees() + 2.0); //-100.0 is just a temporary value that cannot be reached

        if (tx.getDegrees() == 0.0 || ty.getDegrees() == 0.0) {
            linearTX = 0.0;
        }

        m_strafe = m_direction * MathUtil.clamp(pid.calculate(-linearTX, 0.0), -MAX_STRAFE, MAX_STRAFE) * MAX_SPEED; 

         /* 
         * This command utilitzes the swerve drive while it isn't field relative. 
         * The swerve drive returns back to field relative after the command is used.
        */

        ChassisSpeeds speeds =
            new ChassisSpeeds(
                0.0,
                m_strafe,
                0.0);
          m_drive.runVelocity(speeds);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

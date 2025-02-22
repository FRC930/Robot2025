package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.lang.ModuleLayer.Controller;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;

public class alignTx extends Command {
    private Drive m_drive;
    private Vision m_limeLight;
    private int m_cameraIndex;
    private double slope = 0.9; //Needs to be changed to the current slope
    private Controller m_controller;
    private LinearFilter movingAverageFilter;
    private double m_strafe = 0.0;
    private double m_direction;
    private PIDController pid = new PIDController(0.0065, 0.0, 0.0);
    private final double MAX_STRAFE = 0.3; 
    private final double MAX_THROTTLE = 1.0;
    


    public alignTx(Drive drive, Vision limeLight, int cameraIndex) {
        this.m_drive = drive;
        this.m_limeLight = limeLight;
        this.m_cameraIndex = cameraIndex;
        
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_direction = -1.0; //Currently camera in back of robot (go backwards) when camera in front use 1.0
        movingAverageFilter = LinearFilter.movingAverage(3);
        if(m_controller != null) {
           return;
        }

        //Creates the trapezoid profile using the given information
        m_goal = new TrapezoidProfile.State(m_distance, 0.0); //sets the desired state to be the total distance away
        m_setpoint = new TrapezoidProfile.State(0.0, 2.0); //sets the current state at (0,0)
        profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint); //combines everything into the trapezoid profile
        
        // highPassFilter = LinearFilter.highPass(HIGH_PASS_SECONDS, 0.02);
        
    }

    @Override
    public void execute() {
        Rotation2d txLeft = m_limeLight.getTargetX(m_cameraIndex); // degrees left and right from crosshair
        Rotation2d tyLeft = m_limeLight.getTargetY(m_cameraIndex); // degrees up and down from crosshair

        double linearTX = movingAverageFilter.calculate(txLeft <= -100.0 && tyLeft <= -100.0? 0.0 : (tyLeft/slope) - txLeft + 2.0); //-100.0 is just a temporary value that cannot be reached

        if (txLeft == 0.0 || tyLeft == 0.0) {
            linearTX = 0.0;
        }

        m_strafe = m_direction * MathUtil.clamp(pid.calculate(-linearTX, 0.0), -MAX_STRAFE, MAX_STRAFE) * MAX_SPEED; 
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LoggedTunableNumber;

public class alignTx extends Command {
    private Drive m_drive;
    private VisionIOLimelight m_LimeLight;

    public alignTx(Drive drive, VisionIOLimelight LimeLight) {
        this.m_drive = drive;
        this.m_LimeLight = LimeLight;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double tx = m_LimeLight.; // degrees left and right from crosshair
        double ty = m_limeLight.; // degrees up and down from crosshair
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}

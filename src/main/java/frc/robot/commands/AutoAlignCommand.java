package frc.robot.commands;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;

public class AutoAlignCommand extends Command {

    private Drive drivetrain;

    private Pose2d m_targetPose;

    private Function<Pose2d, Pose2d> getTargetPoseFn;

    public static LoggedTunableNumber m_positionTolerance = new LoggedTunableNumber("AutoAlignCommands/Shared/positionErrorToleranceMeters", 0.01);
    public static LoggedTunableNumber m_rotationTolerance = new LoggedTunableNumber("AutoAlignCommands/Shared/rotationErrorToleranceRadians", 0.02);

    public static LoggedTunableNumber spinBound = new LoggedTunableNumber("AutoAlignCommands/Shared/complexSpinBound", 10);
    public static LoggedTunableNumber strafeBound = new LoggedTunableNumber("AutoAlignCommands/Shared/complexThrottleBound", 1.0);
    
    public static LoggedTunableGainsBuilder m_gains = new LoggedTunableGainsBuilder("AutoAlignCommands/Shared/gains/", 6.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    public static LoggedTunableNumber m_maxVelocityTune = new LoggedTunableNumber("AutoAlignCommands/Shared/gains/maxVelMetersPerSecond",3.0);
    public static LoggedTunableNumber m_maxAccelerationTune = new LoggedTunableNumber("AutoAlignCommands/Shared/gains/maxAccMetersPerSecond",3.0);
    public static final double MAX_SPIN = Math.toRadians(180.0);

    private ProfiledPIDController m_pid = new ProfiledPIDController(m_gains.build().kP, m_gains.build().kI ,m_gains.build().kD, new Constraints(m_maxVelocityTune.getAsDouble(), m_maxAccelerationTune.getAsDouble()));
    private PIDController m_spinPID = new PIDController(5.0, 0.0, 0.0);

    private double m_totalDistError;
    private double m_rotationError;

    private MovementMode controlscheme = MovementMode.BOTH_AT_ONCE; // Defaults to simple, changes in some commands

    /**
     * The way the robot will attempt to move to get to target position (order between rotating & moving).
     */
    enum MovementMode {
        BOTH_AT_ONCE, // Try to rotate and lateral move at once
        SUPPRESS_UNTIL_ROTATED, //Supresses lateral movement until the spin is within a certain range
        ;
    }

    /**
     * This command utilitzes the swerve drive while it isn't field relative.
     * The swerve drive returns back to field relative after the command is used.
     * @param getDrivePoseFunction A function that takes a current drivetrain pose and returns a target position.
     * @param drivetrain The Drive class to get the current pose from.
     * @param name The LoggedTunableNumber's (should be) exclusive name
     */
    public AutoAlignCommand(Function<Pose2d, Pose2d> getTargetPoseFunction, Drive drivetrain, String name) {
        this.getTargetPoseFn = getTargetPoseFunction;
        this.drivetrain = drivetrain;
    }

    /**
     * This command utilitzes the swerve drive while it isn't field relative.
     * The swerve drive returns back to field relative after the command is used.
     * @param getDrivePoseFunction A function that takes a current drivetrain pose and returns a target position.
     * @param drivetrain The Drive class to get the current pose from.
     */
    public AutoAlignCommand(Function<Pose2d, Pose2d> getTargetPoseFunction, Drive drivetrain) {
        this(getTargetPoseFunction, drivetrain, "AutoAlign");
    }

    /**
     * Sets the gains to the current values in the LoggedTunableNumbers of this class
     */
    private void resetGains() {
        m_pid = 
            new ProfiledPIDController(m_gains.build().kP, m_gains.build().kI ,m_gains.build().kD, 
                new Constraints(m_maxVelocityTune.getAsDouble(), m_maxAccelerationTune.getAsDouble())
            );
    }

    /**
     * Resets the target pose based on {@link getTargetPoseFn}
     * @return The new target pose
     */
    private Pose2d getNewTargetPose() {
        m_targetPose = getTargetPoseFn.apply(getCurrentPose());
        return m_targetPose;
    }

    /**
     * @return The target pose relative to the robot pose.
     */
    private Pose2d getRelativeTarget() {
        return m_targetPose.relativeTo(getCurrentPose());
    }

    private Pose2d getCurrentPose() {
        return drivetrain.getAutoAlignPose();
    }

    public AutoAlignCommand withControlScheme(MovementMode controlScheme) {
        this.controlscheme = controlScheme;
        return this;
    }

    public void setTargetPoseFn(Function<Pose2d, Pose2d> newFunc) {
        this.getTargetPoseFn = newFunc;
    }

    @Override
    public void initialize() {
        resetGains(); // Reset using logged tunable numbers

        m_targetPose = getNewTargetPose();

        // Use x and y velocity for total velocity
        double velocityTotal = Math.hypot(
            drivetrain.getChassisSpeeds().vxMetersPerSecond, 
            drivetrain.getChassisSpeeds().vyMetersPerSecond);
        
        // Total distance error (should be equiv to hypot of strafeError and throttleError)
        m_totalDistError = getCurrentPose().getTranslation().getDistance(m_targetPose.getTranslation());

        m_pid.reset(m_totalDistError, velocityTotal);
        m_spinPID.reset();
    }

    /* 
    * This command utilitzes the swerve drive while it isn't field relative. 
    * The swerve drive returns back to field relative after the command is used.
    */
    @Override
    public void execute() {

        // Relative (eg. if robot is at (1, 3) and target is (2, 5), relative target is (1, 2))
        Pose2d relativeTargetPose = getRelativeTarget(); 

        // Current error values
        double strafeError = -relativeTargetPose.getY();
        double throttleError = -relativeTargetPose.getX();
        m_rotationError = relativeTargetPose.getRotation().unaryMinus().getRadians();

        // Total distance error (should be equiv to hypot of strafeError and throttleError)
        m_totalDistError = getCurrentPose().getTranslation().getDistance(m_targetPose.getTranslation());

        // Drive PID to 0 from higher number
        double setVelocity = m_pid.calculate(m_totalDistError, 0.0); 

        // Suppress drive if spin error
        if(controlscheme == MovementMode.SUPPRESS_UNTIL_ROTATED) {
            double coef = MathUtil.clamp(
                1.0 - Math.abs((Degrees.convertFrom(m_rotationError, Radians)/spinBound.getAsDouble())), 
                0, 1
            );
            Logger.recordOutput("AutoAlign/coef", coef);
            setVelocity *= coef;
        }

        double angle = Math.atan2(throttleError, strafeError); // angle between strafe direction and direction of end motion

        // m_setVelocity is hypotenuse (actual velocity and direction of motion)
        double strafe = setVelocity * Math.cos(angle); // strafe side of triangle (adjacent side to angle)
        double throttle = setVelocity * Math.sin(angle); // throttle side of triangle (opposite side to angle)
        
        double spin = MathUtil.clamp(m_spinPID.calculate(m_rotationError, 0.0), -MAX_SPIN, MAX_SPIN);

        // Apply the speeds to drivetrain
        ChassisSpeeds speeds =
        new ChassisSpeeds(
            throttle,
            strafe,
            spin);

        drivetrain.runVelocity(speeds);

        Logger.recordOutput("AutoAlign/strafeError", strafeError);
        Logger.recordOutput("AutoAlign/throttleError", throttleError);
        Logger.recordOutput("AutoAlign/rotationError", m_rotationError);
        Logger.recordOutput("AutoAlign/strafe", strafe);
        Logger.recordOutput("AutoAlign/throttle", throttle);
        Logger.recordOutput("AutoAlign/spin", spin);
        Logger.recordOutput("AutoAlign/TargetPose",m_targetPose);
        Logger.recordOutput("AutoAlign/totalDistanceError", m_totalDistError);
        Logger.recordOutput("AutoAlign/setVelocity", setVelocity);

    }

    /**
     * Will return if the robot is within the tolerance of the target pose.
     */
    @Override
    public boolean isFinished() {
        return 
            MathUtil.isNear(m_totalDistError, 0.0, m_positionTolerance.getAsDouble()) 
                && MathUtil.isNear(m_rotationError, 0.0, m_rotationTolerance.getAsDouble()) 
            || !ReefPositionsUtil.getInstance().getIsAutoAligning(); // Shortcircuit if autoalign isn't enabled (safety)
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot unless command was interrupted in order to:
        //  1. Clear trailing speed if position reached within tolerance
        //  2. Keep speed if interrupted for smoother motion (eg. by another auto align command) 
        if (!interrupted) {
            drivetrain.runVelocity(new ChassisSpeeds());
        }
    }
}

package frc.robot.commands;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

public class AutoAlignCommand extends Command {

    private Drive drivetrain;

    private Pose2d targetPose;

    private final Function<Pose2d, Pose2d> getTargetPoseFn;

    //#region TODO get accurate values
    private static LoggedTunableGainsBuilder throttleGains = new LoggedTunableGainsBuilder("AutoAlign/strafeGains/", 6.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    private static LoggedTunableGainsBuilder strafeGains = new LoggedTunableGainsBuilder("AutoAlign/throttleGains/", 4.0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0);
    private static LoggedTunableNumber maxStrafeTune = new LoggedTunableNumber("AutoAlign/strafeGains/maxVelMetersPerSecond",1.0);
    private static LoggedTunableNumber maxThrottleTune = new LoggedTunableNumber("AutoAlign/throttleGains/maxVelMetersPerSecond",1.0);
    private static LoggedTunableNumber maxAccelStrafeTune = new LoggedTunableNumber("AutoAlign/strafeGains/maxAccMetersPerSecond",10.0);
    private static LoggedTunableNumber maxAccelDistanceTune = new LoggedTunableNumber("AutoAlign/throttleGains/maxAccMetersPerSecond",10.0);
    private static LoggedTunableNumber toleranceB = new LoggedTunableNumber("AutoAlign/toleranceB", 0.01);
    private static LoggedTunableNumber toleranceR = new LoggedTunableNumber("AutoAlign/toleranceR", 0.02);
    //#endregion

    private LinearVelocity m_maxStrafe = MetersPerSecond.of(maxStrafeTune.getAsDouble()); 
    private LinearVelocity m_maxThrottle = MetersPerSecond.of(maxThrottleTune.getAsDouble());
    private LinearAcceleration m_maxAccelStrafe = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
    private LinearAcceleration m_maxAccelThrottle = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
    private static final double MAX_SPIN = Math.toRadians(180.0);

    private double m_strafe;
    private double m_throttle;
    private double m_spin;
    private double m_tx;
    private double m_ty;
    private double m_vx;
    private double m_vy;
    private double m_tr;
    private ProfiledPIDController m_strafePID = new ProfiledPIDController(strafeGains.build().kP, strafeGains.build().kI ,strafeGains.build().kD, new Constraints(m_maxStrafe.in(MetersPerSecond), m_maxAccelStrafe.in(MetersPerSecondPerSecond)));
    private ProfiledPIDController m_throttlePID = new ProfiledPIDController(throttleGains.build().kP, throttleGains.build().kI ,throttleGains.build().kD, new Constraints(m_maxThrottle.in(MetersPerSecond), m_maxAccelThrottle.in(MetersPerSecondPerSecond)));
    private PIDController spinPID = new PIDController(5.0, 0.0, 0.0);

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
        m_maxStrafe = MetersPerSecond.of(maxStrafeTune.getAsDouble()); 
        m_maxThrottle = MetersPerSecond.of(maxThrottleTune.getAsDouble());
        m_maxAccelStrafe = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
        m_maxAccelThrottle = MetersPerSecondPerSecond.of(maxAccelDistanceTune.getAsDouble());

        m_strafePID = new ProfiledPIDController(strafeGains.build().kP, strafeGains.build().kI ,strafeGains.build().kD, new Constraints(m_maxStrafe.in(MetersPerSecond), m_maxAccelStrafe.in(MetersPerSecondPerSecond)));
        m_throttlePID = new ProfiledPIDController(throttleGains.build().kP, throttleGains.build().kI ,throttleGains.build().kD, new Constraints(m_maxThrottle.in(MetersPerSecond), m_maxAccelThrottle.in(MetersPerSecondPerSecond)));
    }

    /**
     * Resets the target pose based on {@link getTargetPoseFn}
     * @return The new target pose
     */
    private Pose2d getNewTargetPose() {
        targetPose = getTargetPoseFn.apply(getCurrentPose());
        return targetPose;
    }

    /**
     * @return The target pose relative to the robot pose.
     */
    private Pose2d getRelativeTarget() {
        return targetPose.relativeTo(getCurrentPose());
    }

    private Pose2d getCurrentPose() {
        return drivetrain.getAutoAlignPose();
    }

    @Override
    public void initialize() {
        resetGains();

        this.targetPose = getNewTargetPose();
        Pose2d targetPose_R = getRelativeTarget();

        m_tx = -targetPose_R.getY();
        m_ty = -targetPose_R.getX();
        m_tr = targetPose_R.getRotation().unaryMinus().getRadians();
        m_vx = drivetrain.getChassisSpeeds().vxMetersPerSecond;
        m_vy = drivetrain.getChassisSpeeds().vyMetersPerSecond;

        m_strafePID.reset(m_tx,m_vy);
        m_throttlePID.reset(m_ty,m_vx);
        spinPID.reset();
    }

    /* 
    * This command utilitzes the swerve drive while it isn't field relative. 
    * The swerve drive returns back to field relative after the command is used.
    */
    @Override
    public void execute() {
        Pose2d targetPose_r = getRelativeTarget();

        double distance = getCurrentPose().getTranslation().getDistance(targetPose.getTranslation());

        m_tx = 0.0 - targetPose_r.getY();
        m_ty = 0.0 - targetPose_r.getX();
        m_tr = targetPose_r.getRotation().unaryMinus().getRadians();

        m_strafe = m_strafePID.calculate(m_tx, 0.0); 
        m_throttle = m_throttlePID.calculate(m_ty, 0.0);
        m_spin = MathUtil.clamp(spinPID.calculate(m_tr, 0.0),-MAX_SPIN,MAX_SPIN);

        ChassisSpeeds speeds =
        new ChassisSpeeds(
            m_throttle,
            m_strafe,
            m_spin);
        drivetrain.runVelocity(speeds);

        Logger.recordOutput("AutoAlign/TX", m_tx);
        Logger.recordOutput("AutoAlign/TZ", m_ty);
        Logger.recordOutput("AutoAlign/TR", m_tr);
        Logger.recordOutput("AutoAlign/strafe", m_strafe);
        Logger.recordOutput("AutoAlign/throttle", m_throttle);
        Logger.recordOutput("AutoAlign/spin", m_spin);
        Logger.recordOutput("AutoAlign/TargetPose",targetPose);
        Logger.recordOutput("AutoAlign/distance", distance);
    }

    /**
     * Will return if the robot is within the tolerance of the target pose.
     */
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_tx, 0.0,toleranceR.getAsDouble()) && MathUtil.isNear(m_ty, 0.0,toleranceB.getAsDouble()) && MathUtil.isNear(m_tr, 0.0,(toleranceB.getAsDouble()+toleranceR.getAsDouble())/2.0);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.runVelocity(new ChassisSpeeds());
    }
}

package frc.robot.commands;

import java.util.concurrent.TransferQueue;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ReefPositionsUtil;

public class AutoAlignCommand extends Command {

    private Drive drivetrain;

    private Pose2d targetPose;

    private Function<Pose2d, Pose2d> getTargetPoseFn;

    //#region TODO get accurate values
    public static LoggedTunableGainsBuilder throttleGains = new LoggedTunableGainsBuilder("AutoAlignCommands/Shared/strafeGains/", 6.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    public static LoggedTunableGainsBuilder strafeGains = new LoggedTunableGainsBuilder("AutoAlignCommands/Shared/throttleGains/", 4.0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0);
    public static LoggedTunableNumber maxStrafeTune = new LoggedTunableNumber("AutoAlignCommands/Shared/strafeGains/maxVelMetersPerSecond",3.0);
    public static LoggedTunableNumber maxThrottleTune = new LoggedTunableNumber("AutoAlignCommands/Shared/throttleGains/maxVelMetersPerSecond",3.0);
    public static LoggedTunableNumber maxAccelStrafeTune = new LoggedTunableNumber("AutoAlignCommands/Shared/strafeGains/maxAccMetersPerSecond",5.0);
    public static LoggedTunableNumber maxAccelDistanceTune = new LoggedTunableNumber("AutoAlignCommands/Shared/throttleGains/maxAccMetersPerSecond",5.0);
    public static LoggedTunableNumber toleranceB = new LoggedTunableNumber("AutoAlignCommands/Shared/toleranceB", 0.01);
    public static LoggedTunableNumber toleranceR = new LoggedTunableNumber("AutoAlignCommands/Shared/toleranceR", 0.01);
    public static LoggedTunableNumber spinBound = new LoggedTunableNumber("AutoAlignCommands/Shared/complexSpinBound", 10);
    //#endregion

    public static LinearVelocity m_maxStrafe = MetersPerSecond.of(maxStrafeTune.getAsDouble()); 
    public static LinearVelocity m_maxThrottle = MetersPerSecond.of(maxThrottleTune.getAsDouble());
    public static LinearAcceleration m_maxAccelStrafe = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
    public static LinearAcceleration m_maxAccelThrottle = MetersPerSecondPerSecond.of(maxAccelDistanceTune.getAsDouble());
    public static final double MAX_SPIN = Math.toRadians(180.0);

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

    private Supplier<Transform2d> speedModSupplier;
    private double lastTimestamp = 0.0;

    private ControllerType controlscheme = ControllerType.SIMPLE;

    enum ControllerType {
        SIMPLE, // Behaves the same as the command we've used so far
        COMPLEX_DRIVESUPPRESS //Supresses lateral movement until the spin is within a certain range
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
        this(getTargetPoseFunction, ()->Transform2d.kZero, drivetrain, name);
    }

    public AutoAlignCommand(Function<Pose2d, Pose2d> getTargetPoseFunction, Supplier<Transform2d> speedOffset, Drive drivetrain, String name) {
        this.getTargetPoseFn = getTargetPoseFunction;
        this.drivetrain = drivetrain;
        this.speedModSupplier = speedOffset;
    }

    /**
     * This command utilitzes the swerve drive while it isn't field relative.
     * The swerve drive returns back to field relative after the command is used.
     * @param getDrivePoseFunction A function that takes a current drivetrain pose and returns a target position.
     * @param drivetrain The Drive class to get the current pose from.
     */
    public AutoAlignCommand(Function<Pose2d, Pose2d> getTargetPoseFunction, Supplier<Transform2d> speedOffset, Drive drivetrain) {
        this(getTargetPoseFunction, speedOffset, drivetrain, "AutoAlign");
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

        m_strafePID = new ProfiledPIDController(strafeGains.build().kP, strafeGains.build().kI, strafeGains.build().kD, new Constraints(m_maxStrafe.in(MetersPerSecond), m_maxAccelStrafe.in(MetersPerSecondPerSecond)));
        m_throttlePID = new ProfiledPIDController(throttleGains.build().kP, throttleGains.build().kI, throttleGains.build().kD, new Constraints(m_maxThrottle.in(MetersPerSecond), m_maxAccelThrottle.in(MetersPerSecondPerSecond)));
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

    public AutoAlignCommand withControlScheme(ControllerType controlScheme) {
        this.controlscheme = controlScheme;
        return this;
    }

    public void setTargetPoseFn(Function<Pose2d, Pose2d> newFunc) {
        this.getTargetPoseFn = newFunc;
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

        
        m_strafePID = new ProfiledPIDController(strafeGains.build().kP, strafeGains.build().kI, strafeGains.build().kD, new Constraints(m_maxStrafe.in(MetersPerSecond), m_maxAccelStrafe.in(MetersPerSecondPerSecond)));
        m_throttlePID = new ProfiledPIDController(throttleGains.build().kP, throttleGains.build().kI, throttleGains.build().kD, new Constraints(m_maxThrottle.in(MetersPerSecond), m_maxAccelThrottle.in(MetersPerSecondPerSecond)));

        m_strafePID.reset(m_tx, m_vy);
        m_throttlePID.reset(m_ty, m_vx);
        
        spinPID.reset();
    }

    /* 
    * This command utilitzes the swerve drive while it isn't field relative. 
    * The swerve drive returns back to field relative after the command is used.
    */
    @Override
    public void execute() {
        // targetPose = targetPose.transformBy(speedModSupplier.get().times(Timer.getFPGATimestamp() - lastTimestamp));

        Pose2d targetPose_r = getRelativeTarget();

        double distance = getCurrentPose().getTranslation().getDistance(targetPose.getTranslation());

        m_tx = -targetPose_r.getY();
        m_ty = -targetPose_r.getX();
        m_tr = targetPose_r.getRotation().unaryMinus().getRadians();

        m_strafe = m_strafePID.calculate(m_tx, 0.0); 
        m_throttle = m_throttlePID.calculate(m_ty, 0.0);
        m_spin = MathUtil.clamp(spinPID.calculate(m_tr, 0.0), -MAX_SPIN, MAX_SPIN);

        if(controlscheme == ControllerType.COMPLEX_DRIVESUPPRESS) {
            double coef = MathUtil.clamp(1.0-Math.abs((Degrees.convertFrom(m_tr, Radians)/spinBound.getAsDouble())), 0, 1);
            Logger.recordOutput("AutoAlign/coef", coef);
            m_throttle *= coef;
            m_strafe *= coef;
        }

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


        lastTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Will return if the robot is within the tolerance of the target pose.
     */
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_tx, 0.0,toleranceR.getAsDouble()) && MathUtil.isNear(m_ty, 0.0,toleranceB.getAsDouble()) && MathUtil.isNear(m_tr, 0.0,(toleranceB.getAsDouble()+toleranceR.getAsDouble())/2.0) || !ReefPositionsUtil.getInstance().getIsAutoAligning();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.runVelocity(new ChassisSpeeds());
    }
}

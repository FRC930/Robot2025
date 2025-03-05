package frc.robot.commands;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
    private LoggedTunableGainsBuilder strafeGains = new LoggedTunableGainsBuilder("AutoAlign/strafeGains/", 3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    private LoggedTunableGainsBuilder throttleGains = new LoggedTunableGainsBuilder("AutoAlign/throttleGains/", 7.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    private LoggedTunableNumber maxStrafeTune = new LoggedTunableNumber("AutoAlign/strafeGains/maxVelMetersPerSecond",4);
    private LoggedTunableNumber maxThrottleTune = new LoggedTunableNumber("AutoAlign/throttleGains/maxVelMetersPerSecond",8);
    private LoggedTunableNumber maxAccelStrafeTune = new LoggedTunableNumber("AutoAlign/strafeGains/maxAccMetersPerSecond",128);
    private LoggedTunableNumber maxAccelDistanceTune = new LoggedTunableNumber("AutoAlign/distanceGains/maxAccMetersPerSecond",128);
    private LoggedTunableNumber toleranceB = new LoggedTunableNumber("AutoAlign/toleranceB", 0.1);
    private LoggedTunableNumber toleranceR = new LoggedTunableNumber("AutoAlign/toleranceR", 0.1);
    //#endregion

    private LinearVelocity m_maxStrafe = MetersPerSecond.of(maxStrafeTune.getAsDouble()); 
    private LinearVelocity m_maxThrottle = MetersPerSecond.of(maxThrottleTune.getAsDouble());
    private LinearAcceleration m_maxAccelStrafe = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
    private LinearAcceleration m_maxAccelDist = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
    private static final double MAX_SPIN = Math.toRadians(180.0);

    private double m_strafe;
    private double m_throttle;
    private double m_spin;
    private double m_tx;
    private double m_ty;
    private double m_tr;
    private ProfiledPIDController strafePID = new ProfiledPIDController(throttleGains.build().kP, throttleGains.build().kI ,throttleGains.build().kD, new Constraints(m_maxStrafe.in(MetersPerSecond), m_maxAccelStrafe.in(MetersPerSecondPerSecond)));
    private ProfiledPIDController distancePID = new ProfiledPIDController(strafeGains.build().kP, strafeGains.build().kI ,strafeGains.build().kD, new Constraints(m_maxThrottle.in(MetersPerSecond), m_maxAccelDist.in(MetersPerSecondPerSecond)));
    private PIDController spinPID = new PIDController(5.0, 0.0, 0.0);

    /**
     * @param getDrivePoseFunction A function that takes a current drivetrain pose and returns a target position.
     * @param drivetrain The Drive class to get the current pose from.
     */
    public AutoAlignCommand(Function<Pose2d, Pose2d> getTargetPoseFunction, Drive drivetrain) {
        this.getTargetPoseFn = getTargetPoseFunction;
        this.drivetrain = drivetrain;
    }

    /**
     * Sets the gains to the current values in the LoggedTunableNumbers of this class
     */
    private void resetGains() {
        m_maxStrafe = MetersPerSecond.of(maxStrafeTune.getAsDouble()); 
        m_maxThrottle = MetersPerSecond.of(maxThrottleTune.getAsDouble());
        m_maxAccelStrafe = MetersPerSecondPerSecond.of(maxAccelStrafeTune.getAsDouble());
        m_maxAccelDist = MetersPerSecondPerSecond.of(maxAccelDistanceTune.getAsDouble());

        strafePID = new ProfiledPIDController(throttleGains.build().kP, throttleGains.build().kI ,throttleGains.build().kD, new Constraints(m_maxStrafe.in(MetersPerSecond), m_maxAccelStrafe.in(MetersPerSecondPerSecond)));
        distancePID = new ProfiledPIDController(strafeGains.build().kP, strafeGains.build().kI ,strafeGains.build().kD, new Constraints(m_maxThrottle.in(MetersPerSecond), m_maxAccelDist.in(MetersPerSecondPerSecond)));
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
        return drivetrain.getPose();
    }

    @Override
    public void initialize() {
        resetGains();

        this.targetPose = getNewTargetPose();
        Pose2d targetPose_R = getRelativeTarget();

        m_tx = -targetPose_R.getY();
        m_ty = -targetPose_R.getX();
        m_tr = targetPose_R.getRotation().unaryMinus().getRadians();

        strafePID.reset(m_tx);
        distancePID.reset(m_ty);
        spinPID.reset();
    }

    /* 
    * This command utilitzes the swerve drive while it isn't field relative. 
    * The swerve drive returns back to field relative after the command is used.
    */
    @Override
    public void execute() {
        Pose2d targetPose_r = getRelativeTarget();

        m_tx = -targetPose_r.getY();
        m_ty = -targetPose_r.getX();
        m_tr = targetPose_r.getRotation().unaryMinus().getRadians();

        m_strafe = MathUtil.clamp(strafePID.calculate(m_tx, 0.0), -m_maxStrafe.in(MetersPerSecond), m_maxStrafe.in(MetersPerSecond)); 
        m_throttle = MathUtil.clamp(distancePID.calculate(m_ty, 0.0),-m_maxThrottle.in(MetersPerSecond),m_maxThrottle.in(MetersPerSecond));
        m_spin = MathUtil.clamp(spinPID.calculate(m_tr, 0.0),-MAX_SPIN,MAX_SPIN);

        ChassisSpeeds speeds =
        new ChassisSpeeds(
            m_throttle,
            m_strafe,
            m_spin);
        drivetrain.runVelocity(speeds);

        Logger.recordOutput("AlignTx/TX", m_tx);
        Logger.recordOutput("AlignTx/TZ", m_ty);
        Logger.recordOutput("AlignTx/TR", m_tr);
        Logger.recordOutput("AlignTx/strafe", m_strafe);
        Logger.recordOutput("AlignTx/throttle", m_throttle);
        Logger.recordOutput("AlignTx/spin", m_spin);
        Logger.recordOutput("AlignTx/TargetPose",targetPose);
    }

    /**
     * Will return if the robot is within the tolerance of the target pose.
     */
    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_tx, 0.0,toleranceR.getAsDouble()) && MathUtil.isNear(m_ty, 0.0,toleranceB.getAsDouble()) && MathUtil.isNear(m_tr, 0.0,(toleranceB.getAsDouble()+toleranceR.getAsDouble())/2.0);
    }
}

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

public class RoughAlignToReef extends Command {

    private static int[] targetIdsRed = {
        6,7,8,9,10,11
    };

    private static int[] targetIdsBlue = {
        17,18,19,20,21,22
    };

    private static int[] targetIds;

    private LoggedTunableNumber offsetB = new LoggedTunableNumber("AutoAlign/offsetB", 0.5);
    private LoggedTunableNumber offsetR = new LoggedTunableNumber("AutoAlign/offsetR", 0.2);
    private LoggedTunableNumber toleranceB = new LoggedTunableNumber("AutoAlign/toleranceB", 0.1);
    private LoggedTunableNumber toleranceR = new LoggedTunableNumber("AutoAlign/toleranceR", 0.1);

    private LoggedTunableGainsBuilder strafeGains = new LoggedTunableGainsBuilder("AutoAlign/strafeGains/", 3.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    private LoggedTunableGainsBuilder throttleGains = new LoggedTunableGainsBuilder("AutoAlign/throttleGains/", 7.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    private LoggedTunableNumber maxStrafeTune = new LoggedTunableNumber("AutoAlign/strafeGains/maxVelMetersPerSecond",4);
    private LoggedTunableNumber maxThrottleTune = new LoggedTunableNumber("AutoAlign/throttleGains/maxVelMetersPerSecond",8);
    private LoggedTunableNumber maxAccelStrafeTune = new LoggedTunableNumber("AutoAlign/strafeGains/maxAccMetersPerSecond",128);
    private LoggedTunableNumber maxAccelDistanceTune = new LoggedTunableNumber("AutoAlign/distanceGains/maxAccMetersPerSecond",128);
    
    public enum ReefAlignment {
        LEFT,
        RIGHT,
        CENTER;
    }
    
    private ReefAlignment targetAlignment = ReefAlignment.CENTER;

    private Drive drivetrain;

    private Pose2d drivingPose = Pose2d.kZero;
    
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
     * Finds the closest april tag to a position.
     * 
     * @param pos The Pose2d to find the closest relative tag.
     * @param targets The list of AprilTag IDs to check for.
     * @return The pose of the closest april tag in "targets" to "pos"
     */
    private static Pose2d findClosestPose(Pose2d pos, int[] targets) {
        double minDistance = Double.MAX_VALUE;
        Pose2d target = Pose2d.kZero;
        
        for (int i = 0; i < targets.length; i++) {
            double distance = pos.getTranslation().getDistance(aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).getTranslation().toTranslation2d());
            if (distance < minDistance) {
                target = aprilTagLayout.getTagPose(targets[i]).orElse(Pose3d.kZero).toPose2d();
                minDistance = distance;
            }
        }
        
        return target;
    }

    public RoughAlignToReef(Drive drivetrain, ReefAlignment reefAlignment) {
        this.drivetrain = drivetrain;
        this.targetAlignment = reefAlignment;
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
     * Sets the target pose based on the odometry
     */
    private void resetTargetPose() {
        double localoffsetR = 0;
        switch (targetAlignment) {
            default:
            case RIGHT:
                localoffsetR = offsetR.getAsDouble();
                break;
            case LEFT:
                localoffsetR = offsetR.getAsDouble() * -1;
                break;
            case CENTER:
                localoffsetR = 0;
                break;
        }
        Transform2d offset = new Transform2d(offsetB.getAsDouble(), localoffsetR, Rotation2d.kZero);
        Pose2d closestPose = findClosestPose(drivetrain.getPose(), targetIds);

        Pose2d targetPose = closestPose.transformBy(offset);
        drivingPose = targetPose;
        Logger.recordOutput("ClosestPose",targetPose);
    }

    private Pose2d getCurrentPose() {
        return drivetrain.getPose();
    }

    @Override
    public void initialize() {
        targetIds = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? targetIdsRed : targetIdsBlue ;

        resetGains();
        resetTargetPose();

        Pose2d RobotRelativeTargetPose = drivingPose.relativeTo(getCurrentPose());

        m_tx = -RobotRelativeTargetPose.getY();
        m_ty = -RobotRelativeTargetPose.getX();
        m_tr = RobotRelativeTargetPose.getRotation().unaryMinus().getRadians();

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
        Pose2d RobotRelativeTargetPose = drivingPose.relativeTo(getCurrentPose());

        m_tx = -RobotRelativeTargetPose.getY();
        m_ty = -RobotRelativeTargetPose.getX();
        m_tr = RobotRelativeTargetPose.getRotation().unaryMinus().getRadians();

        m_strafe = MathUtil.clamp(strafePID.calculate(m_tx, 0.0), -m_maxStrafe.in(MetersPerSecond), m_maxStrafe.in(MetersPerSecond)); 
        m_throttle = MathUtil.clamp(distancePID.calculate(m_ty, 0.0),-m_maxThrottle.in(MetersPerSecond),m_maxThrottle.in(MetersPerSecond));
        m_spin = MathUtil.clamp(spinPID.calculate(m_tr, 0.0),-MAX_SPIN,MAX_SPIN);
        // m_spin = spinSupplier.getAsDouble();


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
        Logger.recordOutput("AlignTx/TargetPose",drivingPose);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_tx, 0.0,toleranceR.getAsDouble()) && MathUtil.isNear(m_ty, 0.0,toleranceB.getAsDouble());
    }
    
}

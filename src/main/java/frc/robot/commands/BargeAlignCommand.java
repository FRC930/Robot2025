package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class BargeAlignCommand extends AutoAlignCommand {
    private static final AprilTagFieldLayout aprilTagLayout = Drive.getAprilTagLayout();

    private static final LoggedTunableNumber offsetB = new LoggedTunableNumber("AutoAlignCommands/BargeAlignCommand/offsetB",0.6);
    private static final LoggedTunableNumber maxHorziontalOffset = new LoggedTunableNumber("AutoAlignCommands/BargeAlignCommand/maxHorizontalOffset", 1.5);

    private static final LoggedTunableNumber adjustSpeed = new LoggedTunableNumber("AutoAlignCommands/BargeAlignCommand/adjustSpeed", 1.0);

    private Pose2d targetPose;
    private boolean allianceSidePose;
    private Alliance alliance;
    private Drive drivetrain;

    public BargeAlignCommand(Drive drive, Supplier<Double> strafeControl) {
        super (
          (p)->p,
          ()->new Transform2d(0.0,strafeControl.get() * adjustSpeed.get(),Rotation2d.kZero),
          drive
        );
        
        this.drivetrain = drive;
        this.withControlScheme(ControllerType.COMPLEX_DRIVESUPPRESS);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
      this.alliance = DriverStation.getAlliance().orElse(Alliance.Red);
      Pose2d allianceSide;
      Pose2d nonAllianceSide;
      if(alliance == Alliance.Red) {
        allianceSide = aprilTagLayout.getTagPose(5).orElse(Pose3d.kZero).toPose2d();
        nonAllianceSide = aprilTagLayout.getTagPose(15).orElse(Pose3d.kZero).toPose2d();
      } else {
        allianceSide = aprilTagLayout.getTagPose(14).orElse(Pose3d.kZero).toPose2d();
        nonAllianceSide = aprilTagLayout.getTagPose(4).orElse(Pose3d.kZero).toPose2d();
      } 

      targetPose = drivetrain.getAutoAlignPose().nearest(List.of(allianceSide, nonAllianceSide));
      super.setTargetPoseFn((p) -> getBargeScorePose(p));
      super.initialize();
    }

    private Pose2d getBargeScorePose(Pose2d robotPose) {
        Pose2d robo = drivetrain.getAutoAlignPose();

        Pose2d newPose = new Pose2d(
          targetPose.getX(), 
          MathUtil.clamp(robo.getY(), targetPose.getY()-maxHorziontalOffset.get(), targetPose.getY()+maxHorziontalOffset.get()),
          targetPose.getRotation()
        ).transformBy(new Transform2d(offsetB.getAsDouble(),0.0,Rotation2d.kZero));

        Logger.recordOutput("BargeAlignCommand/targetPose", newPose);
        return newPose ;
  }

  // //Must be killed manually
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
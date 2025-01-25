package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends Vision {

  // Allow robot to turn off updating odometry
  private boolean m_updateOdometryBaseOnApriltags = false;
  private Consumer<Pose2d> m_ResetPose;

  private boolean m_HasRunAutonomous = false;
  private boolean m_HasSeenTags = false;

  private Trigger m_checkIfAllianceChangedTrigger = null;

  private boolean m_isFirstTime = true;

  public AprilTagVision(Consumer<Pose2d> resetPose, VisionConsumer consumer, VisionIO... io) {
    super(consumer, io);
    m_ResetPose = resetPose;

    // try {
    //   aprilTagLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/output.json");
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    //   throw new RuntimeException(e);
    // }
  }

  @Override
  public boolean rejectPose(PoseObservation observation) {
    // If we should reject all apriltags fro being used
    if (!m_updateOdometryBaseOnApriltags) {
      return true;
    }
    return super.rejectPose(observation);
  }

  @Override
  public void periodic() {
    // Log whether updating Odometry based on Apriltags
    Logger.recordOutput("Vision/UpdatingOdometryBasedOnApriltag", m_updateOdometryBaseOnApriltags);
    // TODO If want to not even check all observations (better performance)
    // if (!m_updateOdometryBaseOnApriltags) {
    //    return;
    // }
    super.periodic();
  }

  @Override
  public void addVisionMeasurement(Pose2d pose, double timestamp, Vector<N3> fill) {
    // tell system apriltag was used
    updateTags();
    super.addVisionMeasurement(pose, timestamp, fill);
  }

  /** Tells system an autonomous path was executed */
  public void updateAutonomous() {
    m_HasRunAutonomous = true;
  }

  /** Tells system that we have seen/used an april tag */
  public void updateTags() {
    m_HasSeenTags = true;
  }

  /** Sets the robot position based on the alliance if there is one. */
  public void setRobotPositionBasedOnAlliance() {
    Pose2d temp;
    Pose2d pose;
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    if (optionalAlliance.isPresent()) {
      Alliance alliance = optionalAlliance.get();
      if (alliance == Alliance.Red) {
        temp = aprilTagLayout.getTagPose(7).get().toPose2d();
        pose =
            new Pose2d(
                temp.getX() + 2.0,
                temp.getY(),
                temp.getRotation().plus(Rotation2d.fromDegrees(180.0)));
      } else {
        temp = aprilTagLayout.getTagPose(18).get().toPose2d();
        pose =
            new Pose2d(
                temp.getX() - 2.0,
                temp.getY(),
                temp.getRotation().plus(Rotation2d.fromDegrees(180.0)));
      }

      m_ResetPose.accept(pose);
    }
  }

  /** Create a trigger and waits to update the alliance position when it is available */
  public void createTriggerForSimulation() {
    if (m_checkIfAllianceChangedTrigger == null) {
      m_checkIfAllianceChangedTrigger =
          new Trigger(() -> checkIsAlliancePresent())
              .onTrue(
                  (new InstantCommand(() -> setRobotPositionBasedOnAlliance()))
                      .ignoringDisable(true));
    }
  }

  /**
   * Returns the alliance
   *
   * @return
   */
  private boolean checkIsAlliancePresent() {
    Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
    if (m_isFirstTime) {
      m_isFirstTime = false;
      return false;
    }
    return optionalAlliance.isPresent();
  }

  /**
   * If auto hasn't been run and the robot hasn't seen april tags, it will then update the odoemtry
   * when it enters teleop
   */
  public void updateStartingPosition() {
    if (m_HasRunAutonomous == false && m_HasSeenTags == false) {
      if (Robot.isReal()) {
        setRobotPositionBasedOnAlliance();
      } else {
        createTriggerForSimulation();
      }
    }
  }

  /**
   * Determine if odometry will be updated based on April tag
   *
   * @return ifAprilTagUpdatesOdomotry
   */
  public boolean isUpdateOdometryBaseOnApriltags() {
    return m_updateOdometryBaseOnApriltags;
  }

  /**
   * set whether next april tag will be used for updating odometry
   *
   * @param updateOdometryBasedOnApriltags
   */
  private void setUpdateOdometryBasedOnApriltags(boolean enable) {
    m_updateOdometryBaseOnApriltags = enable;
  }
  /**
   * Disable next if april tag will be used for updating odometry
   *
   * @param disableUpdateOdometryBasedOnApriltags
   */
  public void disableUpdateOdometryBasedOnApriltags() {
    setUpdateOdometryBasedOnApriltags(false);
  }
  /**
   * Disable next if april tag will be used for updating odometry
   *
   * @param enableUpdateOdometryBasedOnApriltags
   */
  public void enableUpdateOdometryBasedOnApriltags() {
    setUpdateOdometryBasedOnApriltags(true);
  }
}

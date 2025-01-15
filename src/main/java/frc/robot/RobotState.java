package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import frc.robot.util.LoggedTunableNumber;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;

public class RobotState extends VirtualSubsystem {
  private static RobotState instance;

  private MutDistance elevatorHeight = Inches.mutable(0);
  private MutAngle shoulderAngle = Degrees.mutable(0);
  private MutAngle elbowAngle = Degrees.mutable(0);
  private MutAngle wristTwist = Degrees.mutable(0);

  private final LoggedTunableNumber elevatorHeightTune =
      new LoggedTunableNumber("robotState/elevatorHeight", 0);
  private final LoggedTunableNumber shoulderAngleTune =
      new LoggedTunableNumber("robotState/shoulderAngle", 0);
  private final LoggedTunableNumber elbowAngleTune =
      new LoggedTunableNumber("robotState/elbowAngle", 0);
  private final LoggedTunableNumber wristTwistTune =
      new LoggedTunableNumber("robotState/wristTwist", 0);

  private MutAngle testStuff = Degrees.mutable(0);

  private final String key;

  private RobotState(String key) {
    this.key = key;
  }

  public static RobotState instance() {
    if (instance == null) {
      instance = new RobotState("measured");
    }
    return instance;
  }

  @Override
  public void periodic() {
    visualize();
  }

  public Distance getElevatorHeight() {
    return elevatorHeight;
  }

  public Angle getShoulderAngle() {
    return shoulderAngle;
  }

  public Angle getElbowAngle() {
    return elbowAngle;
  }

  public Angle getWristTwist() {
    return wristTwist;
  }

  public void setElevatorHeight(Distance elevatorHeight) {
    this.elevatorHeight.mut_replace(elevatorHeight);
  }

  public void setShoulderAngle(Angle shoulderAngle) {
    this.shoulderAngle.mut_replace(shoulderAngle);
  }

  public void setElbowAngle(Angle elbowAngle) {
    this.elbowAngle.mut_replace(elbowAngle);
  }

  public void setWristTwist(Angle wristTwist) {
    this.wristTwist.mut_replace(wristTwist);
  }

  private void visualize() {

    Pose3d elevatorPose =
        new Pose3d(ELEVATOR_ATTACH_OFFSET.getTranslation(), ELEVATOR_ATTACH_OFFSET.getRotation())
            .transformBy(
                new Transform3d(
                    new Translation3d(
                        Meters.zero(), Meters.zero(), Inches.of(elevatorHeightTune.get())),
                    new Rotation3d()));

    Pose3d shoulderPose =
        elevatorPose
            .transformBy(SHOULDER_ATTACH_OFFSET)
            .transformBy(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                        Degrees.of(shoulderAngleTune.get()), Degrees.zero(), Degrees.zero())))
            .transformBy(SHOULDER_PIVOT_OFFSET.inverse());

    Pose3d elbowPose =
        shoulderPose
            .transformBy(ELBOW_ATTACH_OFFSET)
            .transformBy(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                        Degrees.of(elbowAngleTune.get()), Degrees.zero(), Degrees.zero())))
            .transformBy(ELBOW_PIVOT_OFFSET.inverse());

    testStuff.mut_replace(testStuff.plus(Degrees.of(.25)));

    Pose3d wristPose =
        elbowPose
            .transformBy(WRIST_ATTACH_OFFSET)
            .transformBy(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                        Degrees.of(0), Degrees.of(wristTwistTune.get()), Degrees.zero())))
            .transformBy(WRIST_PIVOT_OFFSET.inverse());

    Logger.recordOutput("RobotState/Elevator/" + key, elevatorPose);
    Logger.recordOutput("RobotState/Shoulder/" + key, shoulderPose);
    Logger.recordOutput("RobotState/Elbow/" + key, elbowPose);
    Logger.recordOutput("RobotState/Wrist/" + key, wristPose);
  }

  private static final Transform3d ELEVATOR_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(-1), Inches.of(-0.5), Inches.of(22.6)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(90)));

  private static final Transform3d SHOULDER_PIVOT_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0.0), Inches.of(0.0), Inches.of(0.0)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d SHOULDER_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0.5), Inches.of(-1), Inches.of(17)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d ELBOW_PIVOT_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0.0), Inches.of(0.0), Inches.of(0)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d ELBOW_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0.0), Inches.of(18.0), Inches.of(0.0)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d WRIST_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(-0.4), Inches.of(18.0), Inches.of(0)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d WRIST_PIVOT_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0.0), Inches.of(0.0), Inches.of(0)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0.6)));
}

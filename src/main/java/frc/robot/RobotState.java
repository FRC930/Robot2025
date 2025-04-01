package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.VirtualSubsystem;

import org.littletonrobotics.junction.Logger;

public class RobotState extends VirtualSubsystem {
  private static RobotState instance;

  private MutDistance elevatorHeight = Inches.mutable(0);
  private MutAngle shoulderAngle = Degrees.mutable(0);
  private MutAngle elbowAngle = Degrees.mutable(0);
  private MutAngle wristTwist = Degrees.mutable(0);

  // private final LoggedTunableNumber elevatorHeightTune =
      // new LoggedTunableNumber("robotState/elevatorHeight", 0);
  // private final LoggedTunableNumber shoulderAngleTune =
  //     new LoggedTunableNumber("robotState/elbowAngleZ", 0);
  // private final LoggedTunableNumber elbowAngleTune =
  //     new LoggedTunableNumber("robotState/elbowAngleX", 0);
  // private final LoggedTunableNumber wristTwistTune =
  //     new LoggedTunableNumber("robotState/elbowAngleY", 0);

  private final Mechanism2d primaryMechanism2d;

  private final MechanismRoot2d primaryMechanismRoot; 
  private final MechanismLigament2d shoulderLigament2d;
  private final MechanismLigament2d elbowLigament2d;
  private final MechanismLigament2d elevatorLigament2d;

  private final MechanismRoot2d wristMechanismRoot;
  private final MechanismLigament2d wristMechanismLigament;

  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d baseLigament2d = new MechanismLigament2d("RobotBase", 150, 0, 24, new Color8Bit(Color.kBlue));

  private MutAngle testStuff = Degrees.mutable(0);

  private final String key;

  private RobotState(String key) {
    this.key = key;

    primaryMechanism2d = new Mechanism2d(500, 300);
    elevatorLigament2d = new MechanismLigament2d("ElevatorLigament", elevatorHeight.in(Centimeters), 90);
    shoulderLigament2d = new MechanismLigament2d("ShoulderLigament", Centimeters.convertFrom(15 + 5, Inches), shoulderAngle.in(Degrees));
    elbowLigament2d = new MechanismLigament2d("ElbowLigament", Centimeters.convertFrom(18, Inches), elbowAngle.in(Degrees), 7, new Color8Bit(Color.kOrange));
    wristMechanismLigament = new MechanismLigament2d("WristLigament", Centimeters.convertFrom(18 , Inches), wristTwist.in(Degrees), 5, new Color8Bit(Color.kOrange));

    robotBaseRoot = primaryMechanism2d.getRoot("2dBaseRoot", 225, 20);
    robotBaseRoot.append(baseLigament2d);

    primaryMechanismRoot = primaryMechanism2d.getRoot("2dPrimary", 300, 20);
    primaryMechanismRoot.append(elevatorLigament2d);
    elevatorLigament2d.append(shoulderLigament2d);
    shoulderLigament2d.append(elbowLigament2d);

    wristMechanismRoot = primaryMechanism2d.getRoot("2dWrist", 30, 20);
    wristMechanismRoot.append(wristMechanismLigament);    

    SmartDashboard.putData("Mech2d",primaryMechanism2d);

    // shoulder 18in
    // elbow 15in
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

  public void setElevatorSource(MutDistance elevatorHeight) {
    this.elevatorHeight = elevatorHeight;
  }

  public void setShoulderSource(MutAngle shoulderAngle) {
    this.shoulderAngle = shoulderAngle;
  }

  public void setElbowSource(MutAngle elbowAngle) {
    this.elbowAngle = elbowAngle;
  }

  public void setWristSource(MutAngle wristTwist) {
    this.wristTwist = wristTwist;
  }

  int counter = 0;

  private void visualize() {
    Pose3d elevatorPose =
        new Pose3d(ELEVATOR_ATTACH_OFFSET.getTranslation(), ELEVATOR_ATTACH_OFFSET.getRotation())
            .transformBy(
                new Transform3d(
                    new Translation3d(
                        0, 0, -this.elevatorHeight.in(Meters)),
                    new Rotation3d()));

    Pose3d shoulderPose =
        elevatorPose
            .transformBy(SHOULDER_ATTACH_OFFSET)
            .transformBy(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                      Degrees.of(-136).minus(this.shoulderAngle), Degrees.zero(), Degrees.zero())))
            .transformBy(SHOULDER_PIVOT_OFFSET.inverse());

    Pose3d elbowPose =
        shoulderPose
            .transformBy(ELBOW_ATTACH_OFFSET)
            .transformBy(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                      Degrees.of(66).plus(this.elbowAngle), Degrees.zero(), Degrees.zero())))
            .transformBy(ELBOW_PIVOT_OFFSET.inverse());

    testStuff.mut_replace(testStuff.plus(Degrees.of(.25)));

    Pose3d wristPose =
        elbowPose
            .transformBy(WRIST_ATTACH_OFFSET)
            .transformBy(
                new Transform3d(
                    new Translation3d(),
                    new Rotation3d(
                        Degrees.of(0), Degrees.zero().minus(wristTwist), Degrees.zero())))
            .transformBy(WRIST_PIVOT_OFFSET.inverse());

    double tempShoulderAngle = 90 + shoulderAngle.in(Degrees);
    double tempElbowAngle = - 90 - elbowAngle.in(Degrees);

    elevatorLigament2d.setLength(elevatorHeight.in(Centimeters) + 103.5);
    shoulderLigament2d.setAngle(tempShoulderAngle);
    elbowLigament2d.setAngle(tempElbowAngle);
    wristMechanismLigament.setAngle(wristTwist.in(Degrees));
    
    Logger.recordOutput("RobotState/Elevator/" + key, elevatorPose);
    Logger.recordOutput("RobotState/Shoulder/" + key, shoulderPose);
    Logger.recordOutput("RobotState/Elbow/" + key, elbowPose);
    Logger.recordOutput("RobotState/Wrist/" + key, wristPose);
  }

  private static final Transform3d ELEVATOR_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(2.125), Inches.of(-11.5), Inches.of(3.5)),
          new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(90)));

  private static final Transform3d SHOULDER_PIVOT_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0), Inches.of(0), Inches.of(-38)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d SHOULDER_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(10.5), Inches.of(-1), Inches.of(-34.5)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d ELBOW_PIVOT_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0), Inches.of(-7.44), Inches.of(-22.42)),
          new Rotation3d(Degrees.of(0), Degrees.of(0.0), Degrees.of(0)));

  private static final Transform3d ELBOW_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(1), Inches.of(13.25), Inches.of(-50.25)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d WRIST_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(.375), Inches.of(4.825), Inches.of(-25.5)),
          new Rotation3d(Degrees.of(-200), Degrees.of(0), Degrees.of(0)));

  private static final Transform3d WRIST_PIVOT_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(0.0), Inches.of(0.0), Inches.of(0)),
          new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0)));
}

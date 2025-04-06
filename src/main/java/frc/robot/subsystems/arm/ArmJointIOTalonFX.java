package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.arm.constants.ArmJointConstants;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class ArmJointIOTalonFX implements ArmJointIO {
  public MotionMagicTorqueCurrentFOC Request = null;
  public TalonFX Motor = null;
  public TalonFX FollowerMotor = null;

  public ArmInputs inputs;
  
  private CANcoder cancoder = null;

  private final ArmJointConstants m_Constants;

  private Angle m_setPoint = Angle.ofRelativeUnits(0, Rotations);

  public ArmJointIOTalonFX(ArmJointConstants constants, InvertedValue motorInversion, SensorDirectionValue sensorDirection, NeutralModeValue neutralMode) {
      m_Constants = constants; // Save our constants
      //CANCODER INIT
      if(constants.CanCoderProfile != null) { // If we have a cancoder profile, create our cancoder
        this.cancoder = new CANcoder(constants.CanCoderProfile.id(),constants.CanCoderProfile.bus());
      }
      //MOTOR INIT
      Motor = new TalonFX(constants.LeaderProfile.id(),constants.LeaderProfile.bus()); // Create our motor (We want this to error if we have no motor, so no null checks)
      //FOLLOWER INIT
      if(constants.FollowerProfile != null) { // If we have a follower profile, make it follow
        FollowerMotor = new TalonFX(constants.FollowerProfile.id(),constants.FollowerProfile.bus());
      }
      //REQUEST INIT
      Request = new MotionMagicTorqueCurrentFOC(constants.StartingAngleSim);
      //DEVICE CONFIG
      configureTalons(motorInversion, sensorDirection, neutralMode);
      //MISSING CANCODER?
      if(constants.CanCoderProfile == null) { // If we don't have a cancoder profile, set our starting angle to the one without our cancoder
        this.Motor.setPosition(constants.StartingAngleSim);
      }
  }

  private void configureTalons(InvertedValue motorInversion, SensorDirectionValue cancoderSensorDirection, NeutralModeValue neutralMode) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = neutralMode;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.CurrentLimits.SupplyCurrentLimit = m_Constants.SupplyCurrentLimit.in(Amp);
    cfg.CurrentLimits.StatorCurrentLimit = m_Constants.TorqueCurrentLimit.in(Amp);
    cfg.MotorOutput.Inverted = motorInversion;
    cfg.Feedback.SensorToMechanismRatio = m_Constants.SensorToMechanismGearing;
    cfg.Feedback.RotorToSensorRatio = m_Constants.MotorToSensorGearing;

    if(cancoder != null) {
      cfg.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
      cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

      CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
      cc_cfg.MagnetSensor.MagnetOffset = m_Constants.CanCoderOffset.in(Rotations);//UNIT: ROTATIONS
      cc_cfg.MagnetSensor.SensorDirection = cancoderSensorDirection;
      PhoenixUtil.tryUntilOk(5, () -> cancoder.getConfigurator().apply(cc_cfg));
    }

    if(FollowerMotor != null) {
      TalonFXConfiguration f_cfg = new TalonFXConfiguration();
      f_cfg.MotorOutput.NeutralMode = neutralMode;
      f_cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      f_cfg.CurrentLimits.SupplyCurrentLimit = m_Constants.SupplyCurrentLimit.in(Amp);
      f_cfg.CurrentLimits.StatorCurrentLimit = m_Constants.TorqueCurrentLimit.in(Amp);
      f_cfg.MotorOutput.Inverted = motorInversion;
      f_cfg.Feedback.SensorToMechanismRatio = m_Constants.SensorToMechanismGearing;
      f_cfg.Feedback.RotorToSensorRatio = m_Constants.MotorToSensorGearing;

      PhoenixUtil.tryUntilOk(5, ()->FollowerMotor.getConfigurator().apply(f_cfg));

      FollowerMotor.setControl(new Follower(Motor.getDeviceID(), false));
    }
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
    setGains(Gains.getEmpty(),0);
    setGains(Gains.getEmpty(),1);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.angle.mut_replace(Motor.getPosition().getValue());
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.setPoint.mut_replace(m_setPoint);
    inputs.supplyCurrent.mut_replace(Motor.getSupplyCurrent().getValue());
    
    // inputs.internalSetPoint.mut_replace(Request.getControlInfo());
  }

  @Override
  public void setTarget(Angle target) {
    setTargetWithSlot(target, 0);
  }

  @Override
  public void setTargetWithSlot(Angle target, int slot) {
    Request = Request.withPosition(target).withSlot(slot);
    Motor.setControl(Request);
    m_setPoint = target;
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }

  @Override
  public ArmJointConstants getConstants() {
    return m_Constants;
  }

    @Override
  public void setGains(Gains gains,int slot) {
    SlotConfigs slotConfigs = new SlotConfigs();
    slotConfigs.kP = gains.kP;
    slotConfigs.kI = gains.kI;
    slotConfigs.kD = gains.kD;
    slotConfigs.kS = gains.kS;
    slotConfigs.kG = gains.kG;
    slotConfigs.kV = gains.kV;
    slotConfigs.kA = gains.kA;
    slotConfigs.GravityType = GravityTypeValue.Arm_Cosine;
    slotConfigs.SlotNumber = slot;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(slotConfigs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = gains.kMMV;
    motionMagicConfigs.MotionMagicAcceleration = gains.kMMA;
    motionMagicConfigs.MotionMagicJerk = gains.kMMJ;
    motionMagicConfigs.MotionMagicExpo_kV = gains.kMMEV;
    motionMagicConfigs.MotionMagicExpo_kA = gains.kMMEA;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(motionMagicConfigs));
  }
}

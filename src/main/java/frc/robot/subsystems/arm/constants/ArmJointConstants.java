package frc.robot.subsystems.arm.constants;

import java.util.function.Consumer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.LoggedTunableGainsBuilder;

/**
 * A constants class for instances of the {@link frc.robot.subsystems.arm.ArmJoint} subsystem.
 */
public abstract class ArmJointConstants {
    /**The CanDef profile of the Leader motor.*/
    public CanDef LeaderProfile;
    /**The CanDef profile of the Follower motor.*/
    public CanDef FollowerProfile = null;
    /**The CanDef profile of the CanCoder.*/
    public CanDef CanCoderProfile;

    /**The {@link Gains} structure for a simulation arm.*/
    public Gains SimGains;

    /**The TalonFX gains slot 0. It uses LoggedTunableGainsBuilder for easy resync with Advantage Scope.*/
    public LoggedTunableGainsBuilder TalonFXGainsSlot0;
    /**The TalonFX gains slot 1. It uses LoggedTunableGainsBuilder for easy resync with Advantage Scope.*/
    public LoggedTunableGainsBuilder TalonFXGainsSlot1;

    /**The maximum allowed angular velocity for the arm joint.*/
    public AngularVelocity MaxVelocity;
    /**The maximum allowed angular acceleration for the arm joint.*/
    public AngularAcceleration MaxAcceleration;
    /**The maximum allowed angular jerk (acceleration increase) for the arm joint.*/
    public double MaxJerk;
    /**The limit of Torque Current on the joint motors.*/
    public Current TorqueCurrentLimit;
    /**The limit of Supply Current on the joint motors.*/
    public Current SupplyCurrentLimit;
    /**The Forward Torque limit on the joint motors.*/
    public Current ForwardTorqueLimit;
    /**The Reverse Torque limit on the joint motors.*/
    public Current ReverseTorqueLimit;

    /**The number of motors connected to the arm joint's drive.*/
    public int NumMotors;
    /**The gear ratio from the CanCoder to the Arm Joint. (Sensor : Mechanism)*/
    public double SensorToMechanismGearing;
    /**The gear ratio from the TalonFX to the CanCoder. (Motor : Sensor)*/
    public double MotorToSensorGearing;
    /**The length of the arm section attached to the joint.*/
    public Distance Length;
    /**The weight of the arm section attached to the joint.*/
    public Mass Weight;
    /**The specific motors connected to the arm joint's drive.*/
    public DCMotor Motors;
    /**The maximum absolute angle for the arm joint to move to.*/
    public Angle MaximumAngle;
    /**The minimum absolute angle for the arm joint to move to.*/
    public Angle MinimumAngle;
    /**The angle the arm joint is at on robot startup.*/
    public Angle StartingAngle;

    /**The X position of the arm joint base on the robot.*/
    public Distance XPosition;
    /**The Y position of the arm joint base on the robot.*/
    public Distance YPosition;
    /**The Z position of the arm joint base on the robot.*/
    public Distance ZPosition;
    /**The offset of the CanCoder from it's default 0 angle to the arm's actual 0 angle.*/
    public Angle CanCoderOffset;

    /**The name of the arm joint used in logging.*/
    public String LoggedName;

    /**The function meant to run on sim initialization for the arm.*/
    public Consumer<MutAngle> mechanismSimCallback;
}


package frc.robot.subsystems.arm.constants;

import java.util.function.Consumer;
import java.util.function.Supplier;

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

public abstract class ArmJointConstants {
    public CanDef LeaderProfile;
    public CanDef FollowerProfile = null;
    public CanDef CanCoderProfile;

    public Gains SimGains;

    public LoggedTunableGainsBuilder TalonFXGainsSlot0;
    public LoggedTunableGainsBuilder TalonFXGainsSlot1;

    public AngularVelocity MaxVelocity;
    public AngularAcceleration MaxAcceleration;
    public double MaxJerk;
    public Current TorqueCurrentLimit;
    public Current SupplyCurrentLimit;
    public Current ForwardTorqueLimit;
    public Current ReverseTorqueLimit;

    public int NumMotors;
    public double SensorToMechanismGearing;
    public double MotorToSensorGearing;
    public Distance Length;
    public Mass Weight;
    public DCMotor Motors;
    public Angle MaximumAngle;
    public Angle MinimumAngle;
    public Angle StartingAngle;

    public Distance XPosition;
    public Distance YPosition;
    public Distance ZPosition;
    public Angle CanCoderOffset;

    public String LoggedName;

    public Consumer<MutAngle> mechanismSimCallback;
}


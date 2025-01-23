package frc.robot.subsystems.arm.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.Gains;

public abstract class ArmJointConstants {
    public CanDef LeaderProfile;

    public Gains SimGains;

    public Gains TalonFXGains;

    public AngularVelocity MaxVelocity;
    public AngularAcceleration MaxAcceleration;
    public double MaxJerk;
    public Current TorqueCurrentLimit;
    public Current SupplyCurrentLimit;
    public Current ForwardTorqueLimit;
    public Current ReverseTorqueLimit;

    public int NumMotors;
    public double Gearing;
    public Distance Length;
    public Mass Weight;
    public DCMotor Motors;
    public Angle MaximumAngle;
    public Angle MinimumAngle;
    public Angle StartingAngle;

    public Distance XPosition;
    public Distance YPosition;
    public Distance ZPosition;
    public Angle PitchModifier;

    public String LoggedName;
}


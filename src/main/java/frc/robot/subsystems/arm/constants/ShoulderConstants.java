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

public class ShoulderConstants extends ArmJointConstants {
    public final CanDef LeaderProfile = CanDef.builder().id(0).bus(CanBus.Rio).build();

    public final Gains SimGains =
        Gains.builder().kS(0.0).kG(0.0).kV(1.45).kA(0.0).kP(0.1).kI(0.0).kD(0.0).build();

    public final Gains TalonFXGains =
        Gains.builder().kS(0.0).kG(0.0).kV(0.0).kA(0.0).kP(0.0).kI(0.0).kD(0.0).build();

    public final AngularVelocity MaxVelocity = DegreesPerSecond.of(360);
    public final AngularAcceleration MaxAcceleration = DegreesPerSecondPerSecond.of(360);
    public final double MaxJerk = 0.0;
    public final Current TorqueCurrentLimit = Amps.of(120);
    public final Current SupplyCurrentLimit = Amps.of(40);
    public final Current ForwardTorqueLimit = Amps.of(80);
    public final Current ReverseTorqueLimit = Amps.of(-80);

    public final int NumMotors = 2;
    public final double Gearing = 75;
    public final Distance Length = Inches.of(24.719);
    public final Mass Weight = Pounds.of(11);
    public final DCMotor Motors = DCMotor.getKrakenX60(NumMotors);
    public final Angle MaximumAngle = Degrees.of(360);
    public final Angle MinimumAngle = Degrees.of(0);
    public final Angle StartingAngle = Degrees.zero();

    public final Distance XPosition = Meters.of(0.07);
    public final Distance YPosition = Inches.of(0);
    public final Distance ZPosition = Meters.of(0.377);
    public final Angle PitchModifier = Degrees.of(84);

    public final String LoggedName = "Shoulder";
}

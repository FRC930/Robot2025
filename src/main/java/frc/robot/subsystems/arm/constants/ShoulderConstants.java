package frc.robot.subsystems.arm.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import frc.robot.RobotState;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.Gains;
import frc.robot.util.LoggedTunableGainsBuilder;

public class ShoulderConstants extends ArmJointConstants {
    public ShoulderConstants() {
        this.LeaderProfile = CanDef.builder().id(9).bus(CanBus.CANivore).build();
        this.FollowerProfile = CanDef.builder().id(25).bus(CanBus.CANivore).build();
        this.CanCoderProfile = CanDef.builder().id(22).bus(CanBus.CANivore).build();

        this.SimGains =
            Gains.builder().kS(0.0).kG(0.0).kV(0.0).kA(0.0).kP(0.1).kI(0.0).kD(0.0).build();

        this.LoggedName = "Shoulder";
        this.TalonFXGains = new LoggedTunableGainsBuilder("ArmJoint"+LoggedName+"/Slot0/", 50.0, 0, 0.0, 1.0, 0.0, 6.0, 0.3, 1.0, 2.0, 0, 0, 0);
        this.TalonFXGainsSlot1 = new LoggedTunableGainsBuilder("ArmJoint"+LoggedName+"/Slot1/", 50.0, 0, 0.0, 1.0, 0.0, 6.0, 0.3, 1.0, 2.0, 0, 0, 0);

        this.MaxVelocity = RotationsPerSecond.of(1);
        this.MaxAcceleration = RotationsPerSecondPerSecond.of(5);
        this.MaxJerk = 0.0;
        this.TorqueCurrentLimit = Amps.of(120);
        this.SupplyCurrentLimit = Amps.of(40);
        this.ForwardTorqueLimit = Amps.of(80);
        this.ReverseTorqueLimit = Amps.of(-80);

        this.NumMotors = 1;
        this.SensorToMechanismGearing = 1.0;
        this.MotorToSensorGearing = 89.47368421052633;//Greg moved a tooth on the gear     100.0/38.0*34.0
        this.Length = Inches.of(18);
        this.Weight = Pounds.of(15);
        this.Motors = DCMotor.getKrakenX60(NumMotors);
        this.MaximumAngle = Degrees.of(180);
        this.MinimumAngle = Degrees.of(-180);
        this.StartingAngle = Degrees.of(90);

        this.XPosition = Meters.of(0.07);
        this.YPosition = Inches.of(0);
        this.ZPosition = Meters.of(0.377);
        this.CanCoderOffset = Degrees.of(125.55);
        
        this.mechanismSimCallback = (d) -> {
            RobotState.instance().setShoulderSource(d);
        };
    }
}

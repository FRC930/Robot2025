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

/**
 * An implementation of the {@link ArmJointConstants} for the Elbow.
 * {@inheritDoc}
 */
public class ElbowConstants extends ArmJointConstants {
    public ElbowConstants() {
        this.LeaderProfile = CanDef.builder().id(10).bus(CanBus.CANivore).build();
        this.FollowerProfile = null;
        this.CanCoderProfile = CanDef.builder().id(23).bus(CanBus.CANivore).build();

        this.SimGains =
            Gains.builder()
                //Feed Forward
                .kS(0.0)
                .kG(0.0)
                .kV(0.0)
                .kA(0.0)
                .kP(0.1).kI(0.0).kD(0.0).build();
        
        this.LoggedName = "Elbow";
        this.TalonFXGainsSlot0 = new LoggedTunableGainsBuilder("Gains/"+LoggedName+"/Slot0/", 600.0, 0.0, 60.0, 8.0, 0.0, 0.0, 0.0, 2.0, 10.0, 0.0, 0.0, 0.0);
        this.TalonFXGainsSlot1 = new LoggedTunableGainsBuilder("Gains/"+LoggedName+"/Slot1/", 600.0, 0.0, 60.0, 8.0, 0.0, 0.0, 0.0, 2.0, 10.0, 0.0, 0.0, 0.0);


        this.MaxVelocity = RotationsPerSecond.of(1);
        this.MaxAcceleration = RotationsPerSecondPerSecond.of(5);
        this.MaxJerk = 0.0;
        this.TorqueCurrentLimit = Amps.of(120);
        this.SupplyCurrentLimit = Amps.of(30);
        this.ForwardTorqueLimit = Amps.of(80);
        this.ReverseTorqueLimit = Amps.of(-80);

        this.NumMotors = 1;
        this.SensorToMechanismGearing = 1.0;
        this.MotorToSensorGearing = 50.0;
        this.Length = Inches.of(15);
        this.Weight = Pounds.of(8.5);
        this.Motors = DCMotor.getKrakenX60(NumMotors);
        this.MaximumAngle = Degrees.of(180);
        this.MinimumAngle = Degrees.of(-180);
        this.StartingAngle = Degrees.of(180);

        this.XPosition = Meters.of(0.07);
        this.YPosition = Inches.of(0);
        this.ZPosition = Meters.of(0.377);
        this.CanCoderOffset = Degrees.of(113.5);

        this.LoggedName = "Elbow";

        /**
         * {@inheritDoc}
         */
        this.mechanismSimCallback = (d) -> {
            RobotState.instance().setElbowSource(d);
        };
    }
}

package frc.robot.subsystems.arm.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.RobotState;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.Gains;

public class ElbowConstants extends ArmJointConstants {
    public ElbowConstants() {
        this.LeaderProfile = CanDef.builder().id(10).bus(CanBus.CANivore).build();

        this.SimGains =
            Gains.builder()
                //Feed Forward
                .kS(0.0)
                .kG(0.0)
                .kV(0.0)
                .kA(0.0)
                .kP(0.1).kI(0.0).kD(0.0).build();

        this.TalonFXGains =
            Gains.builder().kS(0.0).kG(0.28).kV(0.0).kA(0.0).kP(50.0).kI(0.0).kD(0.0).build();

        this.MaxVelocity = DegreesPerSecond.of(360);
        this.MaxAcceleration = DegreesPerSecondPerSecond.of(360);
        this.MaxJerk = 0.0;
        this.TorqueCurrentLimit = Amps.of(120);
        this.SupplyCurrentLimit = Amps.of(40);
        this.ForwardTorqueLimit = Amps.of(80);
        this.ReverseTorqueLimit = Amps.of(-80);

        this.NumMotors = 1;
        this.Gearing = 50;
        this.Length = Inches.of(15);
        this.Weight = Pounds.of(8.5);
        this.Motors = DCMotor.getKrakenX60(NumMotors);
        this.MaximumAngle = Degrees.of(360);
        this.MinimumAngle = Degrees.of(0);
        this.StartingAngle = Degrees.zero();

        this.XPosition = Meters.of(0.07);
        this.YPosition = Inches.of(0);
        this.ZPosition = Meters.of(0.377);
        this.PitchModifier = Degrees.of(84);

        this.LoggedName = "Elbow";
        /**
         * This callback defines where we will put our mut_angle on initialize for the purpose of mechanism simulation
         * <p> Whatever function is here gets called once on initialize, and should point to whatever mut_angle source we want to set.
         */
        this.mechanismSimCallback = (d) -> {
            RobotState.instance().setElbowSource(d);
        };
    }
}

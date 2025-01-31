package frc.robot.subsystems.toesies;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.Gains;

public class ToesiesConstants {
  public final CanDef LeaderProfile = CanDef.builder().id(0).bus(CanBus.Rio).build();
  public final AngularVelocity StartingAngularVelocity = null;
  private ToesiesConstants instance;

  public final Gains SimGains =
        Gains.builder().kS(0.0).kG(0.0).kV(1.45).kA(0.0).kP(0.1).kI(0.0).kD(0.0).build();

    public final Gains TalonFXGains =
        Gains.builder().kS(0.0).kG(0.0).kV(0.0).kA(0.0).kP(0.0).kI(0.0).kD(0.0).build();
    
    public final Current TorqueCurrentLimit = Amps.of(120);
    public final Current SupplyCurrentLimit = Amps.of(40);

  private ToesiesConstants() {}

  public ToesiesConstants getInstance() {
    if (instance == null) {
      instance = new ToesiesConstants();
    }

    return instance;
  }
  
    public final String LoggedName = "Toesies";
}

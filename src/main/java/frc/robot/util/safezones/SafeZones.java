package frc.robot.util.safezones;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.BooleanSupplier;

public enum SafeZones {
    SafeToMoveElevator(
        () -> {
            return  (RobotState.instance().getShoulderAngle().in(Degrees) > 30)
            &&      (RobotState.instance().getElbowAngle().in(Degrees) < -110);
        }
    )
    ;

    /**
     * Note that this may be performance intensive, as it forces a check on every safezone every loop.
     */
    public static final boolean LOG_SAFEZONES = true;

    public static void logSafeZones() {
        if(!LOG_SAFEZONES) {
            return;
        }
        for(SafeZones s : SafeZones.values()) {
            Logger.recordOutput("SafeZones/"+s.name(),s.getAsTrigger());
        }
    }

    /**
     * The condition of each safezone
     */
    private Trigger condition;
    
    /**
     * Creates a SafeZone from the given trigger
     * @param condition
     */
    private SafeZones(Trigger condition) {
        this.condition = condition;
    }

    /**
     * Creates a SafeZone from the given boolean supplier
     * @param condition
     */
    private SafeZones(BooleanSupplier condition) {
        this(new Trigger(condition));
    }

    /**
     * Gets a trigger returning when this is safe
     * @return
     */
    public Trigger getAsTrigger() {
        return condition;
    }

    /**
     * Gets a supplier which returns true if this is safe
     * @return
     */
    public BooleanSupplier getAsBooleanSupplier() {
        return condition;
    }

    /**
     * Gets the current value of this safezone
     * @return
     */
    public boolean getCurrent() {
        return condition.getAsBoolean();
    }
}

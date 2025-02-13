package frc.robot.util.commandUtils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.algaeendeffector.AlgaeEndEffector;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

/** 
 * <h1>RobotSystems<h1>
 * <h2>A singleton class for controlling access to our subsystems</h2>
 * <h3>Why:</h3>
 * <p>This is useful because it simplifies access control for the subsystems (Makes it easier to access all of them at once) and forces standardization of how they are called</p>
 * <p>It also fixes the problem with the 2024 robot where CommandFactory methods would rapidly balloon in size because they passed in many subsystem arguments</p>
 * <i>I think the main advantage of this is the standardization of subsystem referencing</i>
 * <h3>USAGE:</h3>
 * <p>To use it, call setSubsystems(...) once in robotContainer after subsystems have been initialized</p>
 * <p>Then, in your CommandFactory, call getInstance() and store the result</p>
 * <p>Use the stored instance to access your subsystems</p>
 */
public class RobotSystems {
    //We need two instances because I want to avoid having to do the whole "RobotSystems" class call before each access.
    //My picture of how the calls will look is:
    //In class params:
    //RobotSystems RS = RobotSystems.getInstance()
    //In methods:
    //RS.shoulder.doSomething()
    //RS.elbow.doSomething()

    private static boolean initialized = false;

    private static ArmJoint i_shoulder;
    private static ArmJoint i_elbow;
    private static Elevator i_elevator;
    private static Wrist    i_wrist;
    private static Intake   i_intake;
    private static CoralEndEffector  i_fingeys;
    private static AlgaeEndEffector  i_toesies;

    public final ArmJoint shoulder;
    public final ArmJoint elbow;
    public final Elevator elevator;
    public final Wrist    wrist;
    public final Intake   intake;
    public final CoralEndEffector  fingeys;
    public final AlgaeEndEffector  toesies;

    private RobotSystems() {
        this.shoulder = i_shoulder;
        this.elbow = i_elbow;
        this.elevator = i_elevator;
        this.wrist = i_wrist;
        this.intake = i_intake;
        this.fingeys = i_fingeys;
        this.toesies = i_toesies;
    }
    
    /**
     * Gets an instance of the RobotSystems
     * @return
     */
    public static RobotSystems getInstance() {
        return new RobotSystems();
    }

    /**
     * Sets the global subsystems. These will be instanced to every RobotSystems class.
     * @param shoulder
     * @param elbow
     * @param elevator
     * @param wrist
     * @param intake
     * @param fingeys
     * @param toesies
     */
    public static void setSubsystems(ArmJoint shoulder, ArmJoint elbow, Elevator elevator, Wrist wrist, Intake intake,
    CoralEndEffector fingeys, AlgaeEndEffector toesies) {
        i_shoulder = shoulder;
        i_elbow = elbow;
        i_elevator = elevator;
        i_wrist = wrist;
        i_intake = intake;
        i_fingeys = fingeys;
        i_toesies = toesies;

        initialized = true;
    }
}

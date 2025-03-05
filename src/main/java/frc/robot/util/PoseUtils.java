package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.Drive;

public class PoseUtils {
    private PoseUtils instance;

    private PoseUtils(Drive drivetrain) {

    }

    public PoseUtils getInstance() {
        if(instance == null) {
            DriverStation.reportError("PoseUtils not initialized before usage!", true);
            return null;
        }
        return instance;
    }

    public PoseUtils getInstance(Drive drivetrain) {
        if(instance == null) {
            instance = new PoseUtils(drivetrain);
        }
        return instance;
    }
}

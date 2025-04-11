package frc.robot.util;

public class PoseUtils {
    private static PoseUtils instance;

    private double m_lastThrottleUsed;
    
    private double m_lastStrafeUsed;

    private PoseUtils() {}

    public static PoseUtils getInstance() {
        if(instance == null) {
            instance = new PoseUtils();
        }
        return instance;
    }

    public double getLastThrottle() {
        return m_lastThrottleUsed;
    }

    public void setLastThrottle(double throttle) {
        m_lastThrottleUsed = throttle;
    }

    public double getLastStrafe() {
        return m_lastStrafeUsed;
    }

    public void setLastStrafe(double strafe) {
        m_lastStrafeUsed = strafe;
    }
}

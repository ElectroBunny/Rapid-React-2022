package frc.robot;

public class RobotMap {
        
    public final static double DIAMETER_WHEEL = 6 * 0.254; //in METERS

    public static final int JOYSTICK_CONTROLLER = 0;
    public static final int XBOX_CONTROLLER = 1;

    // miniCIM motors
	public static final int DRIVE_RIGHT_MASTER = 3;
    public static final int DRIVE_RIGHT_FOLLOWER = 4;
    public static final int DRIVE_LEFT_MASTER = 1;
    public static final int DRIVE_LEFT_FOLLOWER = 2;
    
    // Redline motors
    public static final int VICTOR_CANENET = 10;
    public static final int VICTOR_COLLECTOR = 7;
    public static final int VICTOR_SHOOTER = 8;
    public static final int VICTOR_ARM = 9;

    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;

   
    public static final double BASKET_HEIGHT = 2.64; // in Meters
    
    public static final int RIGHT_SOLENOID_FW = 2; // Bohnots (HEBREW)
    public static final int RIGHT_SOLENOID_BW = 3;
    
    public static final int LEFT_SOLENOID_FW = 0;
    public static final int LEFT_SOLENOID_BW = 1;
}
package robot.subsystems.drivetrain.pure_pursuit;

public class Constants {
    //DRIVING CONSTANTS
    public final static double LOOKAHEAD_DISTANCE = 0.3; //in meters
    public static final double MAX_VELOCITY = 3.762;


    //PATH GENERATION CONSTANTS
    public static final double SPACING_BETWEEN_WAYPOINTS = 0.1524; //meters
    public static final double MAX_ACCEL = 0.1;


    //SMOOTHING CONSTANTS (pure numbers)
    public final static double WEIGHT_SMOOTH = 0.85;
    public final static double WEIGHT_DATA = 1 - WEIGHT_SMOOTH;
    public final static double TOLERANCE = 0.001;

    //VELOCITY CONSTANTS
    public static final double MAX_PATH_VELOCITY = 0.4;
    static final double K_CURVE = 2; //number from 1 to 5

    //DRIVING CONSTANTS (pure numbers)
    public final static double kV = 0.75;
    public final static double kA = 0.000;
    public final static double kP = 0;

    public final static double STOP_SPEED_THRESH = 0.1; //the speed the robot could stop at the end of the path.

    public final static double CYCLE_TIME = 0.02; //cycle time of the roborio

    public final static double ROBOT_WIDTH = 0.6; //width of the robot
}
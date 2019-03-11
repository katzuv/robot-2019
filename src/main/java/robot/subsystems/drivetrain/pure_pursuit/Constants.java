package robot.subsystems.drivetrain.pure_pursuit;

public class Constants {
    //DRIVING CONSTANTS

    public final static double LOOKAHEAD_DISTANCE = 0.55; //in meters
    public static final double MAX_VELOCITY = 3.762;

    //PATH GENERATION CONSTANTS
    public static final double SPACING_BETWEEN_WAYPOINTS = 0.1524; //meters

    //SMOOTHING CONSTANTS (pure numbers)
    public final static double WEIGHT_SMOOTH = 0.85;
    public final static double WEIGHT_DATA = 1 - WEIGHT_SMOOTH;
    public final static double TOLERANCE = 0.001;

    //VELOCITY CONSTANTS
    public static final double MAX_PATH_VELOCITY = 2;
    public static final double MAX_ACCEL = 0.6;
    public static final double K_CURVE = 3; //number from 1 to 5

    //DRIVING CONSTANTS (pure numbers)
    public final static double kV = 1 / MAX_VELOCITY;
    public final static double kA = 0.004;
    public final static double kP = 0.01;

    //Pure pursuit real time
    public final static double MIN_DISTANCE = 0.5;//The minimum distance between the robot to the target point

    public final static double STOP_SPEED_THRESH = 0.1; //the speed the robot could stop at the end of the path.

    public final static double ROBOT_WIDTH = 0.74; //width of the robot
}
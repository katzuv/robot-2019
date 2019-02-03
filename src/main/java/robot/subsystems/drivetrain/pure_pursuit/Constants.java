package robot.subsystems.drivetrain.pure_pursuit;

public class Constants {
    //DRIVING CONSTANTS

    public final static double LOOKAHEAD_DISTANCE = 0.45; //in meters
    public static final double MAX_VELOCITY = 3.762;

    //PATH GENERATION CONSTANTS
    public static final double SPACING_BETWEEN_WAYPOINTS = 0.1524; //meters

    //SMOOTHING CONSTANTS (pure numbers)
    public final static double WEIGHT_SMOOTH = 0.85;
    public final static double WEIGHT_DATA = 1 - WEIGHT_SMOOTH;
    public final static double TOLERANCE = 0.001;

    //VELOCITY CONSTANTS
    public static final double MAX_PATH_VELOCITY = 2;
    public static final double MAX_ACCEL = 0.3;
    public static final double K_CURVE = 3; //number from 1 to 5

    //DRIVING CONSTANTS (pure numbers)
    public final static double kV = 1/MAX_VELOCITY;
    public final static double kA = 0.002;
    public final static double kP = 0.01;

    // fall control
    public final static double K_JERK = 0;
    public final static double ACCELERATION_MISTAKE = 0;
    public final static double MIN_X_DIFFERENCE = 0;
    public final static double ACCELERATION_FIX = 0;

    public final static double STOP_SPEED_THRESH = 0.1; //the speed the robot could stop at the end of the path.

    public final static double CYCLE_TIME = 0.02; //cycle time of the roborio

    public final static double ROBOT_WIDTH = 0.6; //width of the robot

    public final static Point ROLL_AXIS = new Point(0, 0);

    public final static Point MASS_CENTER = new Point(0, 0);

    public final static double CENTER_MASS_TO_AXIS_DISTANCE = Math.sqrt(Math.pow((ROLL_AXIS.y - MASS_CENTER.y), 2) + Math.pow((ROLL_AXIS.x - MASS_CENTER.x), 2));

    public final static double G = 9.81;// gravity
}
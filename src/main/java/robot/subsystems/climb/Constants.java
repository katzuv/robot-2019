package robot.subsystems.climb;
import static robot.Robot.isRobotA;
public class Constants {
    //The climbing mechanism dimensions, given from the robot.
    public static final double LEG_X_DIMENSION = isRobotA ? 0.325 : 0.325;
    public static final double LEG_Y_DIMENSION = isRobotA ? 0.19 : 0.19;

    public static final double BACK_LEG_X_DIMENSION = isRobotA ? 0.35 : 0.35;
    public static final double BACK_LEG_Y_DIMENSION = isRobotA ? 0.09 : 0.09;

    static final double TICKS_PER_METER = isRobotA ? 10000 : 10000; //Should take into account spiral pitch and diameter. todo:change
    static final int TALON_TIMEOUT_MS = 20;
    static final double[] CLIMB_PIDFE = isRobotA ? new double[]{11.567, 0, 0, 7.895, 0} : new double[]{11.567, 0, 0, 7.895, 0}; //Proportional, Intergral, Differential, Feedforward and our own ErrorFix

    public static final double LEVEL_THREE_LEG_LENGTH = 0.47; //Length of leg in meters when climbing. todo:change
    public static final double LEVEL_TWO_LEG_LENGTH = 0.2; //Length of leg in meters when climbing to hab 2. todo:change
    public static final double DRIVE_CLIMB_HEIGHT_THRESH = isRobotA ? 0.1 : 0.1;
    public static final double CLIMB_TOLERANCE = 0.0025; //tolerance in meters from the target position, to allow when stopping the code.
    public static final int MOTION_MAGIC_CRUISE_VELOCITY = (int)(0.05 * TICKS_PER_METER / 10);
    public static final int MOTION_MAGIC_ACCELERATION= (int)(0.05 * TICKS_PER_METER / 10);
    public static final double CALIBRATE_SPEED = 0.23;
    public static final double DRIVE_CLIMB_DRIVETRAIN_DIVISOR = 2.54;

    //Reverse motor directions
    static final boolean WHEEL_TALON_REVERSE = isRobotA ? true : true;
    static final boolean FRONT_LEFT_TALON_REVERSE = isRobotA ? true : true;
    static final boolean FRONT_RIGHT_TALON_REVERSE = isRobotA ? true : true;
    static final boolean BACK_LEFT_TALON_REVERSE = isRobotA ? true : true;
    static final boolean BACK_RIGHT_TALON_REVERSE = isRobotA ? true : true;

    static final boolean FRONT_LEFT_ENCODER_REVERSE = isRobotA ? true : true;
    static final boolean FRONT_RIGHT_ENCODER_REVERSE = isRobotA ? true : true;
    static final boolean BACK_LEFT_ENCODER_REVERSE = isRobotA ? true : true;
    static final boolean BACK_RIGHT_ENCODER_REVERSE = isRobotA ? true : true;

    //reverse hall effects on their respective motor. (normally_open = false, normally_closed = true)
    static final boolean FRONT_LEFT_FORWARD_HALL_REVERSED = isRobotA ? true : true;
    static final boolean FRONT_LEFT_REVERSE_HALL_REVERSED = isRobotA ? true : true;

    static final boolean BACK_RIGHT_FORWARD_HALL_REVERSED = isRobotA ? true : true;
    static final boolean BACK_RIGHT_REVERSE_HALL_REVERSED = isRobotA ? true : true;

    static final boolean BACK_LEFT_REVERSE_HALL_REVERSED = isRobotA ? true : true;
    static final boolean BACK_LEFT_FORWARD_HALL_REVERSED = isRobotA ? true : true;

    static final boolean FRONT_RIGHT_REVERSE_HALL_REVERSED = isRobotA ? true : true;
    static final boolean FRONT_RIGHT_FORWARD_HALL_REVERSED = isRobotA ? true : true;

    static final int BACK_RIGHT_STARTING_OFFSET = -188; //if the leg were to move until the limit switch, what would be its value.
    static final int BACK_LEFT_STARTING_OFFSET = -188; //if the leg were to move until the limit switch, what would be its value.
    static final int FRONT_RIGHT_STARTING_OFFSET = 0; //if the leg were to move until the limit switch, what would be its value.
    static final int FRONT_LEFT_STARTING_OFFSET = 0; //if the leg were to move until the limit switch, what would be its value.

}

package robot.subsystems.climb;
import static robot.Robot.isRobotA;
public class Constants {
    public static final double LEG_EMERGENCY_STOP = 0.035; //delta between each pair of legs which should stop the robots climbing subsystem
    public static final double LEGS_EMERGENCY_OKAY = 0.0025; //delta between each pair of legs in which the mechanism could work again
    public static final double EMERGENCY_FIX_SPEED = 0.25;

    static final double TICKS_PER_METER = isRobotA ? 10000 : 10000; //Should take into account spiral pitch and diameter. todo:change
    static final int TALON_TIMEOUT_MS = 20;
    static final double[] CLIMB_PIDFE = isRobotA ? new double[]{11.567, 0, 0,  7.895, 20} : new double[]{11.567, 0, 0, 7.895, 0}; //Proportional, Intergral, Differential, Feedforward and our own ErrorFix

    public static final double LEVEL_THREE_LEG_LENGTH = 0.50; //Length of leg in meters when climbing. todo:change
    public static final double LEVEL_TWO_LEG_LENGTH = 0.19; //Length of leg in meters when climbing to hab 2. todo:change
    public static final double DRIVE_CLIMB_HEIGHT_THRESH = isRobotA ? 0.1 : 0.1;
    public static final double CLIMB_TOLERANCE = 0.0025; //tolerance in meters from the target position, to allow when stopping the code.
    public static final int MOTION_MAGIC_CRUISE_VELOCITY = (int)(0.16 * TICKS_PER_METER / 10); //in meters per seconds
    public static final int MOTION_MAGIC_ACCELERATION= (int)(0.15 * TICKS_PER_METER / 10); //in meters per second squared
    public static final double DRIVE_CLIMB_DRIVETRAIN_DIVISOR = 2;

    //Reverse motor directions
    static final boolean WHEEL_TALON_REVERSE = isRobotA ? false : false;
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

    public static final double BACK_RIGHT_STARTING_OFFSET = -100/TICKS_PER_METER; //if the leg were to move until the limit switch, what would be its value.
    public static final double BACK_LEFT_STARTING_OFFSET = -100/TICKS_PER_METER; //if the leg were to move until the limit switch, what would be its value.
    public static final double FRONT_RIGHT_STARTING_OFFSET = 0; //if the leg were to move until the limit switch, what would be its value.
    public static final double FRONT_LEFT_STARTING_OFFSET = 0; //if the leg were to move until the limit switch, what would be its value.

}

package robot.subsystems.climb;
import static robot.Robot.isRobotA;
public class Constants {
    //The climbing mechanism dimensions, given from the robot.
    public static final double LEG_X_DIMENSION = isRobotA ? 0.325 : 0.325;
    public static final double LEG_Y_DIMENSION = isRobotA ? 0.19 : 0.19;

    public static final double BACK_LEG_X_DIMENSION = isRobotA ? 0.35 : 0.35;
    public static final double BACK_LEG_Y_DIMENSION = isRobotA ? 0.09 : 0.09;

    public static final double LEVEL_THREE_LEG_LENGTH = 0.5; //Length of leg in meters when climbing. todo:change
    public static final double LEVEL_TWO_LEG_LENGTH = 0.2; //Length of leg in meters when climbing to hab 2. todo:change
    public static final double CLIMB_TOLERANCE = 0.025; //tolerance in meters from the target position, to allow when stopping the code.
    static final double TICKS_PER_METER = isRobotA ? 28571 : 28571; //Should take into account spiral pitch and diameter. todo:change
    static final int TALON_TIMEOUT_MS = 20;
    static final double[] CLIMB_PIDFE = isRobotA ? new double[]{1, 0, 0, 1, 0} : new double[]{0, 0, 0, 0, 0}; //Proportional, Intergral, Differential, Feedforward and our own ErrorFix

    //Reverse motor directions
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
}

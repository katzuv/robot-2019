package robot.subsystems.climb;

public class Constants {
    public static final double CLIMB_TOLERANCE = 0.025;
    public static final double LEG_X_DIMENSION = 0;
    public static final double LEG_Y_DIMENSION = 0;
    public static final double LEVEL_THREE_LEG_LENGTH = 0.5; //Length of leg in meters when climbing. todo:change

    static final double[] CLIMB_PIDFE = {0, 0, 0, 0, 0}; //Proportional, Intergral, Differential, Feedforward and our own ErrorFix
    static final int TALON_TIMEOUT_MS = 20;
    static final double TICKS_PER_METER = 1000; //Should take into account spiral pitch and diameter. todo:change


    static final boolean FRONT_LEFT_TALON_REVERSE = false;
    static final boolean FRONT_RIGHT_TALON_REVERSE = false;
    static final boolean BACK_LEFT_TALON_REVERSE = false;
    static final boolean BACK_RIGHT_TALON_REVERSE = false;

    static final boolean FRONT_LEFT_FORWARD_HALL_REVERSED = false;
    static final boolean FRONT_LEFT_REVERSE_HALL_REVERSED = false;
    static final boolean BACK_RIGHT_FORWARD_HALL_REVERSED = false;
    static final boolean BACK_RIGHT_REVERSE_HALL_REVERSED = false;

    static final boolean BACK_LEFT_REVERSE_HALL_REVERSED = false;
    static final boolean FRONT_RIGHT_REVERSE_HALL_REVERSED = false;
    static final boolean BACK_LEFT_FORWARD_HALL_REVERSED = false;
    static final boolean FRONT_RIGHT_FORWARD_HALL_REVERSED = false;
}

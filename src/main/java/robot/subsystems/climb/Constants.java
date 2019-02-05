package robot.subsystems.climb;

public class Constants {
    public static final boolean UP_LEFT_TALON_REVERSE = false;
    public static final boolean UP_RIGHT_TALON_REVERSE = false;
    public static final boolean DOWN_LEFT_TALON_REVERSE = false;
    public static final boolean DOWN_RIGHT_TALON_REVERSE = false;
    public static final double[] CLIMB_PIDF = {0, 0, 0, 0};
    public static final int TALON_TIMEOUT_MS = 10;
    public static final double TICKS_PER_METER = 1000; //Should take into account spiral pitch and diameter.
    public static final double LEVEL_THREE_LEG_LENGTH = 0.5; //Length of leg in meters when climbing.
    public static final boolean UP_LEFT_FORWARD_HALL_REVERSED = false;
    public static final boolean UP_LEFT_REVERSE_HALL_REVERSED = false;
    public static final boolean DOWN_RIGHT_FORWARD_HALL_REVERSED = false;
    public static final boolean DOWN_RIGHT_REVERSE_HALL_REVERSED = false;
}

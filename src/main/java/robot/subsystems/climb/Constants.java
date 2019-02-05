package robot.subsystems.climb;

public class Constants {
    static final boolean UP_LEFT_TALON_REVERSE = false;
    static final boolean UP_RIGHT_TALON_REVERSE = false;
    static final boolean DOWN_LEFT_TALON_REVERSE = false;
    static final boolean DOWN_RIGHT_TALON_REVERSE = false;
    static final double[] CLIMB_PIDF = {0, 0, 0, 0};
    static final int TALON_TIMEOUT_MS = 10;
    static final double TICKS_PER_METER = 1000; //Should take into account spiral pitch and diameter.
    static final double LEVEL_THREE_LEG_LENGTH = 0.5; //Length of leg in meters when climbing.
    static final boolean UP_LEFT_FORWARD_HALL_REVERSED = false;
    static final boolean UP_LEFT_REVERSE_HALL_REVERSED = false;
    static final boolean DOWN_RIGHT_FORWARD_HALL_REVERSED = false;
    static final boolean DOWN_RIGHT_REVERSE_HALL_REVERSED = false;
}

package robot.subsystems.elevator;

public class Constants {

    public static final double TICKS_PER_METER = 0.0;
    public static final boolean VICTOR_REVERSE = false;
    static final double NOMINAL_OUT_FWD = 0;
    static final int TALON_TIMEOUT_MS = 0;
    static final double DISTANCE_PER_PULSE = 0.0;
    static final double[] LIFT_BOTTOM_DOWN_PIDF = {0, 0, 0, 0};
    static final double[] LIFT_BOTTOM_UP_PIDF = {0, 0, 0, 0};
    static final double[] LIFT_TOP_DOWN_PIDF = {0, 0, 0, 0};
    static final double[] LIFT_TOP_UP_PIDF = {0, 0, 0, 0};
    static final boolean TALON_REVERSE = false;
    static final boolean ENCODER_REVERSED = false;
    static final boolean TOP_HALL_REVERSED = false;
    static final boolean BOTTOM_HALL_REVERSED = false;
    static final double PEAK_OUT_REV = 0;
    static final double NOMINAL_OUT_REV = 0;
    static final double PEAK_OUT_FWD = 0;
    static final double ELEVATOR_TOP_HEIGHT = 2.4;
    static final double ELEVATOR_MID_HEIGHT = 1.2;
    public enum ELEVATOR_STATES { //TODO: organize Constants
        HIGH (2.2),
        MID (1.6),
        CARGO (0.7),
        LOW (0.1),
        BOTTOM(0);

        private final double level_height;

        ELEVATOR_STATES(double height){
            this.level_height = height;
        }

        public double getLevelHeight(){
            return level_height;
        }
    }
}

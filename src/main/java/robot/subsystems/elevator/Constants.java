package robot.subsystems.elevator;

public class Constants {

    //Motor reverse constants:
    static final boolean VICTOR_REVERSE = false;
    static final boolean TALON_REVERSE = false;

    //Limit switch / Magnet hall constants:
    static final boolean TOP_HALL_REVERSED = false;
    static final boolean BOTTOM_HALL_REVERSED = false;

    //Encoder constants:
    static final double TICKS_PER_METER = 0.5;
    static final boolean ENCODER_REVERSED = false;

    /* Talon constants */

    /*
     * Since most config* calls occur during the robot boot sequence, the recommended value for timeoutMs is 10 (ms).
     * This ensures that each config will wait up to 10ms to ensure the configuration was applied correctly,
     * otherwise an error message will appear on the Driver station.
     * This is also the case for setting/homing sensor values.
     *
     * For configuration calls that are done during the robot loop, the recommended value for timeoutMs is zero,
     * which ensures no blocking or checking is performed (identical to the implementation in previous seasons).
     *
     *
     * config* - all the configurations functions starting with the prefix 'config*' and a trailing parameter called timeoutMS.
     */
    static final int TALON_TIMEOUT_MS = 20; //timeout when configuring the robot, if takes longer an error is raised (1)

    /* Nominal Output- The “minimal” or “weakest” motor output allowed if the output is nonzero
     * Peak Output- The “maximal” or “strongest” motor output allowed.
     * These settings are useful to reduce the maximum velocity of the mechanism,
     * and can make tuning the closed-loop simpler.  */
    static final double NOMINAL_OUT_FWD = 0;
    static final double PEAK_OUT_REV = 0;
    static final double NOMINAL_OUT_REV = 0;
    static final double PEAK_OUT_FWD = 0;

    //Mechanical heights of the elevator
    static final double ELEVATOR_TOP_HEIGHT = 2.4; //TODO: get actual heights from mechanics
    static final double ELEVATOR_MID_HEIGHT = 1.2; //TODO: get actual heights from mechanics

    //PIDF values of the elevator
    static final double[] LIFT_BOTTOM_DOWN_PIDF = {0, 0, 0, 0};
    static final double[] LIFT_BOTTOM_UP_PIDF = {0, 0, 0, 0};
    static final double[] LIFT_TOP_DOWN_PIDF = {0, 0, 0, 0};
    static final double[] LIFT_TOP_UP_PIDF = {0, 0, 0, 0};

    public static double ELEVATOR_TOLERANCE = 0.001; //The tolerance in which the elevator will stop when trying to get to a height.

    /**
     * enum storing all height values assigned to their respective height.
     * Currently the heights are: HIGH, MID, CARGO, LOW, BOTTOM.
     */
    public enum ELEVATOR_STATES { //TODO: Find actual heights from mechanics
        HIGH(2.2),
        MID(1.6),
        CARGO(0.7),
        LOW(0.1),
        BOTTOM(0);

        private final double levelHeight;

        ELEVATOR_STATES(double height) {
            this.levelHeight = height;
        }

        public double getLevelHeight() {
            return levelHeight;
        }
    }

    /*
     * (1) Since most config* calls occur during the robot boot sequence, the recommended value for timeoutMs is 10 (ms).
     * This ensures that each config will wait up to 10ms to ensure the configuration was applied correctly,
     * otherwise an error message will appear on the Driver station.
     * This is also the case for setting/homing sensor values.
     *
     * For configuration calls that are done during the robot loop, the recommended value for timeoutMs is zero,
     * which ensures no blocking or checking is performed (identical to the implementation in previous seasons).
     */
}

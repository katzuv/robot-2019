package robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DriverStation;

import static robot.Robot.isRobotA;

public class Constants {

    public static final int START_UNIT = isRobotA ? -720 : -900;
    private static final double NEW_HATCH_DISPLACEMENT = 0.1104;
    public static final double LOWER_DANGER_ZONE = 0.32+Constants.NEW_HATCH_DISPLACEMENT;
    public static final double UPPER_DANGER_ZONE = 0.96+Constants.NEW_HATCH_DISPLACEMENT/2;

    public static final double FLOOR_FEEDFORWARD = 0.04; //The feedforward value when the elevator is at the complete bottom. this is only used to put tension on the strings

    //Encoder constants:
    static final double TICKS_PER_METER = isRobotA ? 25993 : 25993;
    static final boolean ENCODER_REVERSED = isRobotA ? true : true;

    //Motor reverse constants:
    static final boolean VICTOR_REVERSE = isRobotA ? true : true;
    static final boolean TALON_REVERSE = isRobotA ? true : true;

    //Limit switch / Magnet hall constants:
    static final boolean TOP_HALL_REVERSED = isRobotA ? false : false;
    static final boolean BOTTOM_HALL_REVERSED = isRobotA ? false : false;

    //PIDF values of the elevator
    static final double[] LIFT_LOW_UP_PIDF = isRobotA ? new double[]{0.3, 0.0, 0.8, (1023 * 1.0) / 4378.0} : new double[]{0.3, 0.0, 0.8, (1023 * 1.0) / 4378.0};//Units * cruise percent speed / max tested speed

    //Feedforward values for the top and bottom parts of the robot
    static final double FIRST_STAGE_FEEDFORWARD = isRobotA ? 0.17 : 0.17;
    static final double SECOND_STAGE_FEEDFORWARD = isRobotA ? 0.17 : 0.17;

    static final double ELEVATOR_HOLD_IN_PLACE_HEIGHT = isRobotA ? 0.03 : 0.03
    ;
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
    static final int TALON_TIMEOUT_MS = 10; //timeout when configuring the robot, if takes longer an error is raised (1)
    static final int TALON_RUNNING_TIMEOUT_MS = 0; //as seen in the excerpt above, there should be no timeout on the talon in the robot loop.
    //Mechanical heights of the elevator, at its maximum position and semi position(where the elevator splits from one segment to two)
    static final double ELEVATOR_MAX_HEIGHT = 1.56;
    static final double ELEVATOR_MID_HEIGHT = 0.797;
    /* Nominal Output- The "minimal" or "weakest" motor output allowed if the output is nonzero
     * Peak Output- The "maximal" or "strongest" motor output allowed.
     * These settings are useful to reduce the maximum velocity of the mechanism,
     * and can make tuning the closed-loop simpler.  */
    static final double NOMINAL_OUT_FWD = 0.0;
    static final double PEAK_OUT_FWD = 1;
    static final double NOMINAL_OUT_REV = 0;
    static final double PEAK_OUT_REV = -1;
    /* Motion magic speed constants */
    static final int MOTION_MAGIC_ACCELERATION = isRobotA ? (int) (3.2 * TICKS_PER_METER / 10) : (int) (3.2 * TICKS_PER_METER / 10);
    static final int MOTION_MAGIC_CRUISE_SPEED = isRobotA ? (int) (4 * TICKS_PER_METER / 10) : (int) (4 * TICKS_PER_METER / 10);
    public static double ELEVATOR_TOLERANCE = 0.1; //The tolerance in which the elevator will stop when trying to get to a certain height.

    /**
     * enum storing all height values assigned to their respective height.
     * Currently the heights are: HIGH, MID, CARGO, LOW, BOTTOM.
     */
    public enum ELEVATOR_STATES {
        SHIP_HATCH(0.278+Constants.NEW_HATCH_DISPLACEMENT),
        SHIP_CARGO(0.63),
        SHIP_CARGO_BACKWARD(0.78),
        LEVEL1_HATCH(0.278+Constants.NEW_HATCH_DISPLACEMENT),
        LEVEL1_CARGO(0.2),//0.3
        LEVEL1_CARGO_BACKWARD(0.1),
        LEVEL2_HATCH(0.987+Constants.NEW_HATCH_DISPLACEMENT),
        LEVEL2_CARGO(0.9),//0.85
        LEVEL2_CARGO_BACKWARD(0.83),
        LEVEL3_HATCH(1.579+Constants.NEW_HATCH_DISPLACEMENT),
        LEVEL3_CARGO(1.56),//1.48
        LEVEL3_CARGO_BACKWARD(1.5),
        LOADING_STATION(0.278);


        private final double levelHeight;

        ELEVATOR_STATES(double height) {
            this.levelHeight = height;
        }

        public double getLevelHeight() {
            return levelHeight;
        }

        public String getString() {
            if (this == SHIP_HATCH || this == SHIP_CARGO) {
                return "ship";
            } else if (this == LOADING_STATION) {
                return "loading_station";
            }
            else if (DriverStation.getInstance().isAutonomous()) {
                return "sandstorm";
            }
            return "rocket";
        }
    }

}

package robot.subsystems.elevator;

import robot.Robot;
import static robot.Robot.isRobotA;
public class Constants {

    //Encoder constants:
    static final double TICKS_PER_METER = isRobotA ? 25993 : 25993;
    static final boolean ENCODER_REVERSED  = isRobotA ? true : true;

    //Motor reverse constants:
    static final boolean VICTOR_REVERSE = isRobotA ? true : true;
    static final boolean TALON_REVERSE = isRobotA ? true : true;

    //Limit switch / Magnet hall constants:
    static final boolean TOP_HALL_REVERSED = isRobotA ? false : false;
    static final boolean BOTTOM_HALL_REVERSED = isRobotA ? false : false;

    //PIDF values of the elevator
    static final double[] LIFT_LOW_UP_PIDF = isRobotA ? new double[]{0.3, 0.0, 0.8, (1023*1.0)/4378.0} : new double[]{0.3, 0.0, 0.8, (1023*1.0)/4378.0};//Units * cruise percent speed / max tested speed

    //Feedforward values for the top and bottom parts of the robot
    static final double FIRST_STAGE_FEEDFORWARD = isRobotA ? 0.17 : 0.17;
    static final double SECOND_STAGE_FEEDFORWARD = isRobotA ? 0.17 : 0.17;

    static final double ELEVATOR_HOLD_IN_PLACE_HEIGHT = isRobotA ? 0.08 : 0.08;

    public static double ELEVATOR_TOLERANCE = 0.1; //The tolerance in which the elevator will stop when trying to get to a certain height.


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
    static final double ELEVATOR_MAX_HEIGHT = 1.6;
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
    static final int MOTION_MAGIC_ACCELERATION = isRobotA ? (int) (1.7 * TICKS_PER_METER / 10) : (int) (1.7 * TICKS_PER_METER / 10);
    static final int MOTION_MAGIC_CRUISE_SPEED = isRobotA ? (int) (2 * TICKS_PER_METER / 10) : (int) (2 * TICKS_PER_METER / 10);

    /**
     * enum storing all height values assigned to their respective height.
     * Currently the heights are: HIGH, MID, CARGO, LOW, BOTTOM.
     */
    public enum ELEVATOR_STATES {
        SHIP_HATCH(0.278),
        SHIP_CARGO(0.3),
        LEVEL1_HATCH(0.278),
        LEVEL1_CARGO(0.3),
        LEVEL2_HATCH(0.987),
        LEVEL2_CARGO(0.85),
        LEVEL3_HATCH(1.579),
        LEVEL3_CARGO(1.48),
        LOADING_STATION(0.278);

        private final double levelHeight;

        ELEVATOR_STATES(double height) {
            this.levelHeight = height;
        }

        public double getLevelHeight() {
            return levelHeight;
        }
    }

}

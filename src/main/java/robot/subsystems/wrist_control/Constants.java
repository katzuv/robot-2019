package robot.subsystems.wrist_control;

import static robot.Robot.isRobotA;

public class Constants {
    public static final double WRIST_JUMP_ANGLE = 30;
    public static final double DEFAULT_TIMEOUT = 1.5; //If no default is specified, and if its not disabled, set to this default timeout when trying to reach a setpoint
    final static double CARGO_IN_VOLTAGE = isRobotA ? 1.2 : 1.2; //TODO: change
    final static double TICKS_PER_DEGREE = isRobotA ? 11.73333333333333333333333 * 4 : 11.73333333333333333333333 * 4; // (reduction=66/16) * (ticks_per_revolution=1024) / 360deg

    final static boolean WRIST_MOTOR_REVERSED = isRobotA ? false : false;
    final static boolean SENSOR_PHASE = isRobotA ? false : false;

    final static boolean SOFT_LIMIT_OVERRIDE = isRobotA ? false : false;
    final static boolean LIMIT_SWITCH_OVERRIDE = isRobotA ? false : false;

    final static boolean REVERSE_NORMALLY_CLOSED = isRobotA ? false : false;
    final static boolean FORWARD_NORMALLY_CLOSED = isRobotA ? false : false;

    static boolean PROXIMITY_DISABLED = false; //When true, the proximity value is hard set to 0
    static boolean WRIST_FORWARD_DROP_DISABLED = false; //When true, the wrist doesn't drop when forward

    public final static double TRIGGER_MINIMAL_VALUE = 0.2; //Minimal value when using triggers. sort of like a deadband

    final static int TALON_TIME_OUT = isRobotA ? 10 : 10;

    public static final int MOTION_MAGIC_ACCELERATION = isRobotA ? 3500 : 2468;
    public static final int CRUISE_VELOCITY  = isRobotA ? 6000 : 3000;

    final static double kP = isRobotA ? 0.9 : 0.7;
    final static double kD = isRobotA ? 70 : 150;
    final static double kF = isRobotA ? 0.52 : 0.52;

    final static int IZone = isRobotA ? 200 : 50;
    final static double kI = isRobotA ? 0.003 : 0.00197;

    final static double PEAK_OUTPUT_FORWARD = isRobotA ? 0.6 : 0.6;
    final static double PEAK_OUTPUT_REVERSE = isRobotA ? -0.6 : -0.6;


    final static boolean IS_MAG_ENCODER_RELATIVE = isRobotA ? true : true;

    /* Stall current constants */
    public static final double DROP_WRIST_ANGLE = 3;
    public static final double COM_ANGLE = 7.77 + 30; //the angle of the center of mass at the initial angle of the wrist. ~ in robot A the starting angle was calculated.
    static final double PEAK_PERCENT_COMPENSATION = 0.38; //The percent output that is given to the wrist to hold it in place at its peak.
    public static final double ZERO_ANGLE_COMPENSATION = 0.025;

    //idea for a stall current calculation when hatches are inside.
    private final static double WRIST_WEIGHT = 1;
    private final static double HATCH_WEIGHT = 0.1;
    private final static double CARGO_WEIGHT = 0.05;

    public static final double WRIST_DANGER_ANGLE = 30;

    final static double HATCH_MULTIPLIER = (WRIST_WEIGHT + HATCH_WEIGHT) / WRIST_WEIGHT;
    final static double CARGO_MULTIPLIER = (WRIST_WEIGHT + CARGO_WEIGHT) / WRIST_WEIGHT;
    public enum WRIST_ANGLES {//TODO: remove useless angles for new wrist.
        INITIAL(0),
        UP(90 - COM_ANGLE),
        FORWARD(150),
        SHIP(137),
        SHIP_BACKWARD(0),
        LEVEL_1(106),
        LEVEL_2(106),
        LEVEL_3(113),
        LEVEL_1_BACKWARD(0),
        LEVEL_2_BACKWARD(0),
        LEVEL_3_BACKWARD(0),
        INTAKE(150),
        MAXIMAL(150);
        private final double wristAngle;

        WRIST_ANGLES(double angle) {
            this.wristAngle = angle;
        }

        public double getValue() {
            return wristAngle;
        }
    }

    public enum GRIPPER_SPEED {
        INTAKE(-0.75),
        SHIP(0.9),
        SHIP_BACKWARD(0.4),
        LEVEL_1(0.9),
        LEVEL_2(0.9),
        LEVEL_3(0.9),
        LEVEL_1_BACKWARD(1),
        LEVEL_2_BACKWARD(1),
        LEVEL_3_BACKWARD(1);
        private final double gripperSpeed;

        GRIPPER_SPEED(double height) {
            this.gripperSpeed = height;
        }

        public double getValue() {
            return gripperSpeed;
        }
    }
}

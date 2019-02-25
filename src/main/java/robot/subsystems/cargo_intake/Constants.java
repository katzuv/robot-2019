package robot.subsystems.cargo_intake;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import robot.Robot;
import static robot.Robot.isRobotA;

public class Constants {
    public final static double GRIPPER_INTAKE_SPEED = isRobotA ? -0.75 : -0.75; //should always be negative
    public final static double GRIPPER_SHOOT_SPEED = isRobotA ? 0.9 : 0.9; //should always be positive

    public static final int MOTION_MAGIC_ACCELERATION = isRobotA ? 1000 : 1000;
    public static final int CRUISE_VELOCITY  = isRobotA ? 2100 : 1600;

    final static double kP = isRobotA ? 0.6 : 0.6;
    final static double kD = isRobotA ? 150 : 100;
    final static double kF = isRobotA ? 0.48 : 0.48;

    final static int IZone = isRobotA ? 50 : 50;
    final static double kI = isRobotA ? 0.001 : 0.001;


    final static double TICKS_PER_DEGREE = isRobotA ? 11.73333333333333333333333 * 4 : 11.73333333333333333333333 * 4; // (reduction=66/16) * (ticks_per_revolution=1024) / 360deg

    final static boolean WRIST_MOTOR_REVERSED = isRobotA ? false : false;
    final static boolean REVERSE_NORMALLY_CLOSED = isRobotA ? false : false;
    final static boolean FORWARD_NORMALLY_CLOSED = isRobotA ? false : false;


    final static int TALON_TIME_OUT = isRobotA ? 10 : 10;

    final static double CARGO_IN_VOLTAGE = isRobotA ? 1 : 0.64;
    final static boolean SENSOR_PHASE = isRobotA ? false : false;

    final static boolean SOFT_LIMIT_OVERRIDE =isRobotA ? true : true;
    final static boolean LIMIT_SWITCH_OVERRIDE =isRobotA ? true : false;

    final static double PEAK_OUTPUT_FORWARD = isRobotA ? 0.7 : 0.6;
    final static double PEAK_OUTPUT_REVERSE = isRobotA ? -0.7 : -0.6;


    final static boolean IS_MAG_ENCODER_RELATIVE = isRobotA ? true : true;

    public enum WRIST_ANGLES{
        INITIAL(0),
        UP(82.75),
        SHIP(108),
        SHIP_BACKWARD(108),
        LEVEL_1(108),
        LEVEL_2(108),
        LEVEL_3(108),
        LEVEL_1_BACKWARD(108),
        LEVEL_2_BACKWARD(108),
        LEVEL_3_BACKWARD(108),
        INTAKE(165),
        MAXIMAL(230);
        private final double wristAngle;
        WRIST_ANGLES(double height) {
            this.wristAngle = height;
        }
        public double getValue() {
            return wristAngle;
        }
    }
}


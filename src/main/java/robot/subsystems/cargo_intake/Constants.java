package robot.subsystems.cargo_intake;

import robot.Robot;
import static robot.Robot.isRobotA;

public class Constants {
    public final static double GRIPPER_WHEELS_SPEED = isRobotA ? 0.75 : 0.75;

    public static final int MOTION_MAGIC_ACCELERATION = isRobotA ? 1000 : 1000;
    public static final int CRUISE_VELOCITY  = isRobotA ? 1600 : 1600;
    public static final double MIN_STALL_ANGLE = 5;

    final static double kP = isRobotA ? 0.6 : 0.6;
    final static double kD = isRobotA ? 100 : 100;
    final static double kF = isRobotA ? 0.48 : 0.48;

    final static int IZone = isRobotA ? 50 : 50;
    final static double kI = isRobotA ? 0.001 : 0.001;


    final static double TICKS_PER_DEGREE = isRobotA ? 11.73333333333333333333333 * 4 : 11.73333333333333333333333 * 4; // (reduction=66/16) * (ticks_per_revolution=1024) / 360deg

    final static boolean WRIST_MOTOR_REVERSED = isRobotA ? false : true;
    final static boolean REVERSE_NORMALLY_CLOSED = isRobotA ? false : false;
    final static boolean FORWARD_NORMALLY_CLOSED = isRobotA ? false : false;


    final static int TALON_TIME_OUT = isRobotA ? 10 : 10;

    final static double CARGO_IN_VOLTAGE = isRobotA ? 0.64 : 0.64;
    final static boolean SENSOR_PHASE = isRobotA ? false : false;

    public enum WRIST_ANGLES{
        INITIAL(0),
        UP(82.75),
        SHOOTING(135),
        INTAKE(172),
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


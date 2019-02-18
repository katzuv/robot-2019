package robot.subsystems.cargo_intake;

import robot.Robot;

public class Constants {
    public static final int MOTION_MAGIC_ACCELERATION;
    public static final int CRUISE_VELOCITY;

    final static double kP;
    final static double kD;
    final static double kF;

    final static int IZone;
    final static double kI;


    final static double TICKS_PER_DEGREE; // (reduction=66/16) * (ticks_per_revolution=1024) / 360deg

    final static boolean WRIST_MOTOR_REVERSED;
    final static boolean REVERSE_NORMALLY_CLOSED;
    final static boolean FORWARD_NORMALLY_CLOSED;


    final static int TALON_TIME_OUT;

    final static double CARGO_IN_VOLTAGE;
    final static boolean SENSOR_PHASE;

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

    static {
        if (Robot.isRobotA) {
            MOTION_MAGIC_ACCELERATION = 1000;
            CRUISE_VELOCITY = 1600;

            kP = 0.6;
            kD = 100;
            kF = 0.48;

            IZone = 50;
            kI = 0.001;


            TICKS_PER_DEGREE = 11.73333333333333333333333 * 4; // (reduction=66/16) * (ticks_per_revolution=1024) / 360deg

            WRIST_MOTOR_REVERSED = true;
            REVERSE_NORMALLY_CLOSED = false;
            FORWARD_NORMALLY_CLOSED = false;


            TALON_TIME_OUT = 10;

            CARGO_IN_VOLTAGE = 0.64;
            SENSOR_PHASE = false;
        } else { //ROBOT B CONSTANTS
            MOTION_MAGIC_ACCELERATION = 1000;
            CRUISE_VELOCITY = 1600;

            kP = 0.6;
            kD = 100;
            kF = 0.48;

            IZone = 50;
            kI = 0.001;


            TICKS_PER_DEGREE = 11.73333333333333333333333 * 4; // (reduction=66/16) * (ticks_per_revolution=1024) / 360deg

            WRIST_MOTOR_REVERSED = true;
            REVERSE_NORMALLY_CLOSED = true;
            FORWARD_NORMALLY_CLOSED = true;


            TALON_TIME_OUT = 10;

            CARGO_IN_VOLTAGE = 0.64;
            SENSOR_PHASE = true;
        }
    }
}


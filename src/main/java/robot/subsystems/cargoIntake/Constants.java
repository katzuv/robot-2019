package robot.subsystems.cargoIntake;

public class Constants {
    public final static double GRIPPER_WHEELS_SPEED = 0.75;
    public final static double INITIAL_ANGLE = 0;//initial angle of the wrist
    public final static double INTAKE_ANGLE = 0;
    public final static double FOLDED_ANGLE = 165;//folded angle represents the angle in which the wrist is folded back inside the robot (number felt cute might delete later)

    public static final int MOTION_MAGIC_ACCELERATION = 1000;
    public static final int CRUISE_VELOCITY = 1600;

    final static double kP = 0.6;
    final static double kD = 100;
    final static double kF = 0.48;

    final static int IZone = 50;
    final static double kI = 0.001;


    final static double TICKS_PER_DEGREE = 11.73333333333333333333333 * 4; // (reduction=66/16) * (ticks_per_revolution=1024) / 360deg

    final static boolean WRIST_MOTOR_REVERSED = true;
    final static boolean REVERSE_NORMALLY_CLOSED = false;
    final static boolean FORWARD_NORMALLY_CLOSED = false;


    final static int TALON_TIME_OUT = 10;

    final static double CARGO_IN_VOLTAGE = 0.64;
    final static boolean SENSOR_PHASE = false;
}

package robot.subsystems.cargoIntake;

public class Constants {
    public final static double kP = 0;
    public final static double kI = 0;
    public final static double kD = 0;
    public final static double kF = 0;
    public final static int TALON_TIME_OUT = 0;
    public final static double TICKS_PER_DEGREE = 11.73333333333333333333333*4; // (reduction=66/16) * (ticks_per_revolution=1024) / 360deg
    public final static int SLOT_IDX = 0;
    public final static int PID_LOOP_IDX = 0;
    public final static double WRIST_RADIUS = 0;
    public final static double CARGO_IN_VOLTAGE = 2.5;//felt cute might delete later
    public final static double GRIPPER_SPEED = 0.75;
    public final static double INTAKE_ANGLE = 0;
    public final static double FOLDED_ANGLE = 165;//folded angle represents the angle in which the wrist is folded back inside the robot (number felt cute might delete later)
    public final static boolean WRIST_LIMIT_REVESED = true;//might need to be changed
    public final static double INITIAL_ANGLE = 0;//initial angle of the wrist
    public static final int IZone = 0;
    public static final boolean SENSOR_PHASE = false;
    public static final boolean WRIST_MOTOR_REVERSED = false;
}

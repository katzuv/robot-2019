package robot.subsystems.cargo_intake;

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

    final static boolean WRIST_MOTOR_REVERSED = false;
    final static boolean WRIST_LIMIT_REVESED = true;//might need to be changed

    final static int TALON_TIME_OUT = 10;

    final static double CARGO_IN_VOLTAGE = 0.64;
    final static boolean SENSOR_PHASE = false;

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

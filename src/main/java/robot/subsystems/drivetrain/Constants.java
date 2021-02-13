package robot.subsystems.drivetrain;

import static robot.Robot.isRobotA;

public class Constants {
    public final static double ROBOT_WIDTH = 0.74; //the distance between the left and right wheels on the robot

    public static final boolean LEFT_ENCODER_REVERSED = isRobotA ? false : false;
    public static final boolean RIGHT_ENCODER_REVERSED = isRobotA ? false : false;

    public static final double LEFT_TICKS_PER_METER = isRobotA ? 2138.7750882690398 : 2138.7750882690398; // measured average distance (robot a), [1m / (diameter=0.1524 * pi)] * (ticks_per_meter=1024s) (robot b)
    public static final double RIGHT_TICKS_PER_METER = isRobotA ? 2138.7750882690398 : 2138.7750882690398;

    public static final int TALON_RUNNING_TIMEOUT_MS = isRobotA ? 0 : 0;
    public static final int TALON_TIMEOUT_MS = isRobotA ? 10 : 10;

    public static final double[] PIDFLeft = isRobotA ? new double[]{1.5, 0, 10, 1.3} : new double[]{1.5, 0, 10, 1.3};
    public static final double[] PIDFRight = isRobotA ? new double[]{1.5, 0, 10, 1.3} : new double[]{1.5, 0, 10, 1.3};
    public static final double distanceFromEnd = 2;
    public static final double ENDING_TOLERANCE = 0.05;
    public static final double MIN_AIM = 0.07;
    public static final double MOTION_CRUISE_VELOCITY = 2; //Motion acceleration in M/S
    public static final double MOTION_ACCELERATION = 2; //Motion acceleration in M/S^2
    public static final double RAMSETE_PEAK_VELOCITY = 2.5;
    public static final double RAMSETE_PEAK_ACCELERATION = 2.5;
    public static final double VISION_SPEED = isRobotA ? 0.3 : 0.3;
    public static final double SLOW_JOYSTICK_SPEED = 1; //multiplied by joystick value, keep at 1 for no changes.
    public static final double kZeta = 0.8;
    public static final double kBeta = isRobotA ? 2 : 2;
    public static final double angleKp = isRobotA ? 1.6 : 1.37;
    public static final double visionOffset = 1.2;
    //Model constants
    public static final double kRobotMass = 55;
    public static final double kRobotAngularDrag = 12;
    public static final double kRobotMomentOfIntertia = 10;
    public static final double FORWARD_SPEED_CONSTANT = 0;
    // (8 inches), the encoder sends 226
    // pulses every 360 degree turn
    static final boolean LEFT_MASTER_REVERSED = isRobotA ? false : false;
    static final boolean LEFT_SLAVE1_REVERSED = isRobotA ? false : false;
    static final boolean LEFT_SLAVE2_REVERSED = isRobotA ? false : false;
    static final boolean RIGHT_MASTER_REVERSED = isRobotA ? true : true;
    static final boolean RIGHT_SLAVE1_REVERSED = isRobotA ? true : true;
    static final boolean RIGHT_SLAVE2_REVERSED = isRobotA ? true : true;
    private static final double ACCELERATION_CONSTRAINT = 1.2;
    private static final double VELOCITY_CONSTRAINT = 3;
    private static final double RECTANGLE_1 = 4;
    private static final double RECTANGLE_2 = 7;
    private static final double RECTANGLE_3 = 8;
    private static final double RECTANGLE_4 = 20;

    public static double HATCH_TARGET_DISTANCE = 1.2;
    public static double CARGO_TARGET_DISTANCE = 1.45;

    public static double[] PIDVisionTurn = isRobotA ? new double[]{0.025, 0, 0.017} : new double[]{0.02, 0.0001, 0.01};

    public static double[] PIDAngularVelocity = isRobotA ? new double[]{0.45, 0.0003, 0.1} : new double[]{0.3, 0, 0};

    public static double[] PIDVisionForward = isRobotA ? new double[]{0.4, 0.0015, 0.1} : new double[]{0, 0, 0};
    public static double pathAngleKp = isRobotA ? 2.3 : 2;

    public static double[] TURNING_PID = isRobotA ? new double[]{0.15, 0, 0.44} : new double[]{0.15, 0, 0.44};
    public static double TURNING_PEAK = 3;

    public static double kDriveLeftKv = 0.7256 / 3.28;
    public static double kDriveLeftKa = 0.2766 / 3.28;
    public static double kDriveLeftKs = 0.8616 / 3.28;
    public static double kDriveRightKv = 0.7340 / 3.28;
    public static double kDriveRightKa = 0.2865 / 3.28;
    public static double kDriveRightKs = 0.8486 / 3.28;


}

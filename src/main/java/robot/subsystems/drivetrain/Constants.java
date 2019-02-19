package robot.subsystems.drivetrain;

import static robot.Robot.isRobotA;

public class Constants {
    public final static double ROBOT_WIDTH = 0.74; //the distance between the left and right wheels on the robot
    public static final double MAX_RATE = isRobotA ? 0.3 : 0.3;
    public static final double DISTANCE_PER_PULSE = isRobotA ? (0.2032 * Math.PI) / 231 : (0.2032 * Math.PI) / 231;  //diameter of the wheel is 0.2032 meters (8 inches), the encoder sends 226 pulses every 360 degree turn
    public static final boolean LEFT_ENCODER_REVERSED = isRobotA ? false : false;
    public static final boolean RIGHT_ENCODER_REVERSED = isRobotA ? false : false;
    public static final double TICKS_PER_METER = isRobotA ? 2138.7750882690398 : 2138.7750882690398; // [1m / (diameter=0.1524 * pi)] * (ticks_per_meter=1024s)
    public static final int TALON_RUNNING_TIMEOUT_MS = isRobotA ? 0 : 0;
    public static final int TALON_TIMEOUT_MS = isRobotA ? 10 : 10;
    public static final double[] PIDF = isRobotA ? new double[]{0, 0, 0, 0} : new double[]{0, 0, 0, 0};
                                                    // (8 inches), the encoder sends 226
                                                    // pulses every 360 degree turn
                                                    static final boolean LEFT_MASTER_REVERSED = isRobotA ? false : false;
    static final boolean LEFT_SLAVE1_REVERSED = isRobotA ? false : false;
    static final boolean LEFT_SLAVE2_REVERSED = isRobotA ? false : false;
    static final boolean RIGHT_MASTER_REVERSED = isRobotA ? true : true;
    static final boolean RIGHT_SLAVE1_REVERSED = isRobotA ? true : true;
    static final boolean RIGHT_SLAVE2_REVERSED = isRobotA ? true : true;
}

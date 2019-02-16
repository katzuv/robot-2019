package robot.subsystems.drivetrain;

public class Constants {
    public final static double ROBOT_WIDTH = 0.6; //the distance between the left and right wheels on the robot
    public static final double MAX_RATE = 0.3;
    public static final double DISTANCE_PER_PULSE = (0.2032 * Math.PI) / 231;//diameter of the wheel is 0.2032 meters (8 inches), the encoder sends 226 pulses every 360 degree turn
    // (8 inches), the encoder sends 226
                                                                              // pulses every 360 degree turn
    static final boolean LEFT_MASTER_REVERSED = false;
    static final boolean LEFT_SLAVE1_REVERSED = false;
    static final boolean LEFT_SLAVE2_REVERSED = false;
    static final boolean RIGHT_MASTER_REVERSED = true;
    static final boolean RIGHT_SLAVE1_REVERSED = true;
    static final boolean RIGHT_SLAVE2_REVERSED = true;

    public static final boolean LEFT_ENCODER_REVERSED = false;
    public static final boolean RIGHT_ENCODER_REVERSED = false;
    public static final double TICKS_PER_METER = 2138.7750882690398; // [1m / (diameter=0.1254 * pi)] * (ticks_per_meter=1024s)
    public static final int TALON_RUNNING_TIMEOUT_MS = 0;
    public static final int TALON_TIMEOUT_MS = 10;
    public static final double[] PIDF = { 0, 0, 0, 0 };
}
package robot.subsystems.drivetrain;

import robot.Robot;

public class Constants {
    public final static double ROBOT_WIDTH; //the distance between the left and right wheels on the robot
    public static final double MAX_RATE;
    public static final double DISTANCE_PER_PULSE;  //diameter of the wheel is 0.2032 meters (8 inches), the encoder sends 226 pulses every 360 degree turn
                                                    // (8 inches), the encoder sends 226
                                                    // pulses every 360 degree turn
    static final boolean LEFT_MASTER_REVERSED;
    static final boolean LEFT_SLAVE1_REVERSED;
    static final boolean LEFT_SLAVE2_REVERSED;
    static final boolean RIGHT_MASTER_REVERSED;
    static final boolean RIGHT_SLAVE1_REVERSED;
    static final boolean RIGHT_SLAVE2_REVERSED;

    public static final boolean LEFT_ENCODER_REVERSED;
    public static final boolean RIGHT_ENCODER_REVERSED;
    public static final double TICKS_PER_METER; // [1m / (diameter=0.1524 * pi)] * (ticks_per_meter=1024s)
    public static final int TALON_RUNNING_TIMEOUT_MS;
    public static final int TALON_TIMEOUT_MS;
    public static final double[] PIDF = new double[4];

    static {
        if (Robot.isRobotA) {
            ROBOT_WIDTH = 0.6;
            MAX_RATE = 0.3;
            DISTANCE_PER_PULSE = (0.2032 * Math.PI) / 231;
            LEFT_MASTER_REVERSED = false;
            LEFT_SLAVE1_REVERSED = false;
            LEFT_SLAVE2_REVERSED = false;
            RIGHT_MASTER_REVERSED = true;
            RIGHT_SLAVE1_REVERSED = true;
            RIGHT_SLAVE2_REVERSED = true;

            LEFT_ENCODER_REVERSED = false;
            RIGHT_ENCODER_REVERSED = false;

            TICKS_PER_METER = 2138.7750882690398;
            TALON_RUNNING_TIMEOUT_MS = 0;
            TALON_TIMEOUT_MS = 10;

            PIDF[0] = 0; //P
            PIDF[1] = 0; //I
            PIDF[2] = 0; //D
            PIDF[3] = 0; //F
        }
        else{
            ROBOT_WIDTH = 0.6;
            MAX_RATE = 0.3;
            DISTANCE_PER_PULSE = (0.2032 * Math.PI) / 231;
            LEFT_MASTER_REVERSED = false;
            LEFT_SLAVE1_REVERSED = false;
            LEFT_SLAVE2_REVERSED = false;
            RIGHT_MASTER_REVERSED = true;
            RIGHT_SLAVE1_REVERSED = true;
            RIGHT_SLAVE2_REVERSED = true;

            LEFT_ENCODER_REVERSED = false;
            RIGHT_ENCODER_REVERSED = false;

            TICKS_PER_METER = 2138.7750882690398;
            TALON_RUNNING_TIMEOUT_MS = 0;
            TALON_TIMEOUT_MS = 10;

            PIDF[0] = 0; //P
            PIDF[1] = 0; //I
            PIDF[2] = 0; //D
            PIDF[3] = 0; //F
        }
    }
}

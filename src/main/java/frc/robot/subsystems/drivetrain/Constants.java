package frc.robot.subsystems.drivetrain;

public class Constants {
    public final static double ROBOT_WIDTH = 0.6; //the distance between the left and right wheels on the robot

    static final boolean LEFT_REVERSED = false;
    static final boolean RIGHT_REVERSED = true;
    public static final double MAX_RATE = 0.3;
    public static final double DISTANCE_PER_PULSE = (0.2032 * Math.PI) / 231;//diameter of the wheel is 0.2032 meters (8 inches), the encoder sends 226 pulses every 360 degree turn

}
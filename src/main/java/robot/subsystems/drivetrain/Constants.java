package robot.subsystems.drivetrain;

public class Constants {
    static final boolean LEFT_REVERSED = false;
    static final boolean RIGHT_REVERSED = true;
    public static final double MAX_RATE = 0.3;
    static final double DISTANCE_PER_PULSE = (0.2032 * Math.PI) / 231;//diameter of the wheel is 0.2032 meters (8 inches), the encoder sends 226 pulses every 360 degree turn

}
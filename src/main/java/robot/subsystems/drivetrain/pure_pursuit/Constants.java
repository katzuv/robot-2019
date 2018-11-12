package robot.subsystems.drivetrain.pure_pursuit;

public class Constants {
    public final static double WEIGHT_SMOOTH = 0.85;
    public final static double WEIGHT_DATA = 1-WEIGHT_SMOOTH;
    public final static double TOLERANCE = 0.001;
    public final static double LOOKAHEAD_DISTANCE = 0.3;
    public final static double TRACK_WIDTH = 3;
    public final static double Kv = 0;
    public final static double Ka = 0;
    public final static double Kp = 0;
    public final static double STOP_SPEED_THRESH = 0.1;
}

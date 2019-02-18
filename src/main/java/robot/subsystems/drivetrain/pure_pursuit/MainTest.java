package robot.subsystems.drivetrain.pure_pursuit;

public class MainTest {


    public static void main(String[] args) {
        Path path = new Path();
        path.appendWaypoint(new Waypoint(0, 0));
        path.appendWaypoint(new Waypoint(0.5, 0));
        System.out.println("BASIC PATH:");
        System.out.println(path);
        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        System.out.println("AFTER ALL GENERATION:");
        System.out.println(path);

    }
}
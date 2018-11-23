package robot.subsystems.drivetrain.pure_pursuit;

public class MainTest {


    public static void main(String[] args) {
        Path path = new Path();
        path.appendWaypoint(new Waypoint(0, 0));
        path.appendWaypoint(new Waypoint(7, 1.5));
        path.appendWaypoint(new Waypoint(10, 5));
        path = path.generateFillPoint();
        path = path.generateSmoothing(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE);
        path.generateCurvature();
        path.generateDistance();
        path.generateVelocity(Constants.MAX_ACCEL);
        path.getWaypoint(0).setSpeed(path.getWaypoint(1).getSpeed()/2);
        System.out.println(path);

    }
}
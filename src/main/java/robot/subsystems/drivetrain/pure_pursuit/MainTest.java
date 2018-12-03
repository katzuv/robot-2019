package robot.subsystems.drivetrain.pure_pursuit;

class MainTest {


    public static void main(String[] args) {
        Path path = new Path();
        path.appendWaypoint(new Waypoint(0, 0));
        path.appendWaypoint(new Waypoint(1.2, 0.7));
        path.appendWaypoint(new Waypoint(2, 1));
        path.appendWaypoint(new Waypoint(3, 1.5));
        path = path.generateFillPoint();
        path = path.generateSmoothing(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE);
        path.generateCurvature();
        path.generateDistance();
        path.generateVelocity(Constants.MAX_ACCEL);
        path.getWaypoint(0).setSpeed(path.getWaypoint(1).getSpeed()/2);
        System.out.println(path);

    }

}
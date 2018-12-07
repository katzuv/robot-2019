package robot.subsystems.drivetrain.pure_pursuit;

class MainTest {


    public static void main(String[] args) {
        Path path = new Path();
        path.appendWaypoint(new Waypoint(0, 0));
        path.appendWaypoint(new Waypoint(0, 1));
        path.appendWaypoint(new Waypoint(4, 1));
        System.out.println("BASIC PATH:");
        System.out.println(path);
        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        System.out.println("AFTER ALL GENERATION:");
        System.out.println(path);

        System.out.println(Math.tan(Math.toRadians(90)));

    }

}
package robot.subsystems.drivetrain.pure_pursuit;

class MainTest {
    public static void main(String[] args) {
        Path path = new Path(new Waypoint[]{new Waypoint(0, 0), new Waypoint(0.5, 0.5), new Waypoint(1, 1)});
        path.generateAll(0.5, 0.5, 0.001, 2, 2);
        System.out.println(path);
    }
}

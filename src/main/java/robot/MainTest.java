package robot;

import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

public class MainTest {


    public static void main(String[] args) {
        Path path = new Path();
        path.append(new Waypoint(0, 0));
        path.append(new Waypoint(5, 1));
        path.append(new Waypoint (-1, -2));
        path.append(new Waypoint (0, 2));
        System.out.println(path);
        path = path.generateFillPoint();
        System.out.println(path);
//        Path newPath = path.generateSmoothing(Constants);
        System.out.println("path2");
        System.out.println(path);


    }
}
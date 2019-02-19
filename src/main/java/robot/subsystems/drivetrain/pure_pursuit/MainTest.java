package robot.subsystems.drivetrain.pure_pursuit;

public class MainTest {


    public static void main(String[] args) {
        //Path path = new Path();
        //path.generateCircleFillPoints(new Point(-2,0), new Point(0,2), new Point(0, 0),-0.1, path);
        Point start = new Point(0,0);
        Point end = new Point(1,1);
        double start_angle =0;

        double end_angle = 10;
        double radius =0.5;
        Path path = new Path(start,start_angle,end,end_angle,radius,1,1);
        System.out.println(String.format("Start: %s - %f\nEnd: %s - %f\nRadius: %f", start, start_angle, end, end_angle, radius));
        System.out.println(path);

    }
}
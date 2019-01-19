package robot.subsystems.drivetrain.pure_pursuit;

public class MainTest {


    public static void main(String[] args) {
        Path path = new Path(new Point(0,1), 90, new Point(3,4), -20, 0.5);
        //Path path = new Path();
        //path.generateCircleFillPoints(new Point(-2,0), new Point(0,2), new Point(0, 0),-0.1, path);
        System.out.println(path);

    }
}
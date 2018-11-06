package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Drivetrain;

/**
 *
 */
public class PurePursue extends Command {
    private double lastLeftEncoder;
    private double lastRightEncoder;
    private double initAngle;
    private Path path;
    private Point currentPoint;
    private Drivetrain drive;

    /**
     * @param path
     */
    public PurePursue(Path path) {
        drive = Robot.drivetrain;
        this.path = path;
        currentPoint = new Point(0, 0);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        lastLeftEncoder = drive.getLeftDistance();
        lastRightEncoder = drive.getRightDistance();
        initAngle = Robot.navx.getAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        updatePoint();


    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    private void updatePoint() {
        //change in (change left encoder value + change in right encoder value)/2
        double distance = ((drive.getLeftDistance() - lastLeftEncoder) + (drive.getRightDistance() - lastRightEncoder)) / 2;
        currentPoint.setX(currentPoint.getX() + distance * Math.cos(drive.getAngle() * (Math.PI / 180.0)));
        currentPoint.setY(currentPoint.getY() + distance * Math.sin(drive.getAngle() * (Math.PI / 180.0)));

        lastLeftEncoder = drive.getLeftDistance();
        lastRightEncoder = drive.getRightDistance();
    }
    //Only Part of the function! doesn't run function through all of the path
    //https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899

    /**
     * @param ref
     * @param lookahead
     * @param point1
     * @param point2
     * @return
     */
    private Point findNearPath(Point ref, double lookahead, Waypoint point1, Waypoint point2) {
        Vector p = new Vector(point2, point1);
        Vector f = new Vector(point1, ref);

        //p*p + 2*f*p + f*f - r*r = 0
        double a = p.dot(p);
        double b = 2 * f.dot(p);
        double c = f.dot(f) - lookahead * lookahead;
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0)
            return null; //means that the circle doesnt reach the line
        else {
            discriminant = Math.sqrt(discriminant);
            double opt1 = (-b - discriminant) / (2 * a);
            double opt2 = (-b + discriminant) / (2 * a);
            if (opt1 >= 0 && opt1 <= 1) {
                return p.multiply(opt1).add(point1);
            }
            if (opt2 >= 0 && opt2 <= 1)
                return p.multiply(opt2).add(point1);
        }
        return null; //means that the segment is entirely inside the line.
    }

    /**
     * @return
     */
    private Point findLookaheadInPath() {
        return null;
    }

    /**
     * @param path the path that function work on
     * @return point that closest to the robot position
     * @author orel
     */
    private Point closestPoint(Path path) {
        Point closest = path.get(0);
        for (int i = 1; i < path.length(); i++) {

            if (Point.distance(this.currentPoint, path.get(i)) < Point.distance(this.currentPoint, closest)) {
                closest = path.get(i);
            }


        }
        return closest;
    }

    /**
     * @author orel
     * @param path current path
     * @return the curvature for point
     */
    private double curvatureCalculate(Path path) {
        double x = closestPoint(path).getX() - currentPoint.getX();
        double y = closestPoint(path).getY() - currentPoint.getY();
        double L = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double radius = Math.pow(L, 2) / 2 * x;
        double d = radius - x;
        if (radius == 0) {
            return Double.POSITIVE_INFINITY;
        } else {
            return 1 / radius;
        }
    }

    /**
     * @author orel
     * @param path current path
     * @return the distance from the lockahead positive mean to right negative mean to left
     */
    private double distance_lockahead(Path path){
        double tan_robot_angle = closestPoint(path).getY()-currentPoint.getX()/closestPoint(path).getX()-currentPoint.getX();
        double a = -tan_robot_angle;
        double b = 1;
        double c =tan_robot_angle* (currentPoint.getX()-currentPoint.getY());
        return a*findLookaheadInPath().getX()+ b*findLookaheadInPath().getY()+c/Math.sqrt(Math.pow(a,2)+Math.pow(b,2));
    }

    /**
     * @return
     */
    public double getRightSpeed() {
        return 0;
    }

    /**
     * @return
     */
    public double getLeftSpeed() {
        return 0;
    }

}
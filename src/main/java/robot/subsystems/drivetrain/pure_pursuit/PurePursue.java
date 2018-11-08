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
     * A command class.
     * @param path the Path class that the robot is going to follow
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
        /*
        Main methods:
            updatePoint()
            findLookaheadInPath()
            closest Point
         */

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

    /**
     * @author Paulo
     * Updates the Point object holding the robots x and y cooridinates
     */
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
     * This method finds the furthest point in a segment that is in a specified distance from the robot.
     * @param ref       Center point of the robot
     * @param lookahead lookahead distance (units of measurements are the same as those stored in the points)
     * @param point1    start of a segment
     * @param point2    end of a segment
     * @return returns the furthest point
     * @author Lior barkai
     * @author Paulo
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
    }

    /**
     * Uses the 'FindNearPath' method on all segments to find the closest point.
     * Checks for the next intersection thats index is higher than the current lookahead point.
     * @return the Lookahead Point.
     */
    private Point findLookaheadInPath() {
        return null;
    }

    /**
     * Runs through a specified path and finds the closest waypoint to the robot.
     * @param path the path that this method work on
     * @return the closest point to the robots position
     * @author orel
     */
    private Point closestPoint(Path path) {
        Point closest = path.getWaypoint(0).copy();
        for (int i = 1; i < path.length(); i++) {

            if (Point.distance(this.currentPoint, path.getWaypoint(i)) < Point.distance(this.currentPoint, closest)) {
                closest = path.getWaypoint(i);
            }


        }
        return closest;
    }

    /**
     * Calculates the curvature between the tangent of the robot and its lookahead point.
     * @param path current path
     * @return The curvature from the tangent of the robot to the setpoint
     * @author orel
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
     * Calculates how far the robots tangent is from the lookahead point.
     * This method is used to check whether the robot is left or right of the point aswell.
     * @author orel
     * @param path current path
     * @return The X axis distance (relative to robot) from the lookahead, right side being positive.
     */
    private double distance_lookahead(Path path) {
        double tan_robot_angle = closestPoint(path).getY() - currentPoint.getX() / closestPoint(path).getX() - currentPoint.getX();
        double a = -tan_robot_angle;
        double b = 1;
        double c = tan_robot_angle * (currentPoint.getX() - currentPoint.getY());
        return a * findLookaheadInPath(path).getX() + b * findLookaheadInPath(path).getY() + c / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
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
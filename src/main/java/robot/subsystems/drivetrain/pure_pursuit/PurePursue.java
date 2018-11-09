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
    private Waypoint currentPoint;
    private Waypoint lastPoint;
    private Drivetrain drive;
    private Point currentLookahead;
    private double lastLookaheadDistance;

    /**
     * A command class.
     *
     * @param path the Path class that the robot is going to follow
     */
    public PurePursue(Path path) {
        drive = Robot.drivetrain;
        this.path = path;
        currentPoint = new Waypoint(0, 0);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        lastLeftEncoder = drive.getLeftDistance();
        lastRightEncoder = drive.getRightDistance();
        initAngle = drive.getAngle();
        currentLookahead = path.getWaypoint(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        updatePoint();
        updateLookaheadInPath(path);
        /*
        Main methods:
            updatePoint()
            updateLookaheadInPath()
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
        lastPoint = currentPoint;
        currentPoint.setX(currentPoint.getX() + distance * Math.cos(drive.getAngle() * (Math.PI / 180.0)));
        currentPoint.setY(currentPoint.getY() + distance * Math.sin(drive.getAngle() * (Math.PI / 180.0)));

        lastLeftEncoder = drive.getLeftDistance();
        lastRightEncoder = drive.getRightDistance();
    }
    //Only Part of the function! doesn't run function through all of the path
    //https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899

    /**
     * This method finds the furthest point in a segment that is in a specified distance from the robot.
     *
     * @param ref       Center point of the robot
     * @param lookahead lookahead distance (units of measurements are the same as those stored in the points)
     * @param point1    start of a segment
     * @param point2    end of a segment
     * @return returns the furthest point
     * @author Lior barkai
     * @author Paulo
     */
    private Waypoint findNearPath(Point ref, double lookahead, Waypoint point1, Waypoint point2) {
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
            Waypoint newLookaheadPoint = null;
            discriminant = Math.sqrt(discriminant);
            double opt1 = (-b - discriminant) / (2 * a);
            double opt2 = (-b + discriminant) / (2 * a);
            if (opt1 >= 0 && opt1 <= 1) {
                newLookaheadPoint = p.multiply(opt1).add(point1);
            }
            if (opt2 >= 0 && opt2 <= 1)
                if (newLookaheadPoint != null) {
                    if (opt2 > opt1)
                        newLookaheadPoint = p.multiply(opt2).add(point1);
                } else
                    newLookaheadPoint = p.multiply(opt2).add(point1);
            return newLookaheadPoint;
        }
    }

    /**
     * Uses the 'FindNearPath' method on all segments to find the closest point.
     * Checks for the next intersection thats index is higher than the current lookahead point.
     *
     * @return the Lookahead Point.
     */
    private void updateLookaheadInPath(Path path) {
        for (int i = 0; i < path.length() - 1; i++) {
            Waypoint wp = findNearPath(currentPoint, Constants.LOOKAHEAD_DISTANCE, path.getWaypoint(i), path.getWaypoint(i + 1));
            if (Point.distance(wp, path.getWaypoint(i)) + path.getWaypoint(i).getDistance() > lastLookaheadDistance) {
                lastLookaheadDistance = Point.distance(wp, path.getWaypoint(i)) + path.getWaypoint(i).getDistance();
                currentLookahead = wp;
                return;
            }
        }
    }

    /**
     * Runs through a specified path and finds the closest waypoint to the robot.
     *
     * @param path the path that this method work on
     * @return the closest point to the robots position
     * @author orel
     */
    private Waypoint closestPoint(Path path) {
        Waypoint closest = path.getWaypoint(0).copy();
        for (int i = 1; i < path.length(); i++) {

            if (Point.distance(this.currentPoint, path.getWaypoint(i)) < Point.distance(this.currentPoint, closest)) {
                closest = path.getWaypoint(i);
            }


        }
        return closest;
    }

    /**
     * Calculates the curvature between the tangent of the robot and its lookahead point.
     *
     * @param path current path
     * @return The curvature from the tangent of the robot to the setpoint
     * @author orel
     * @author Paulo
     */
    private double curvatureCalculate(Path path) {
        double x = distance_lookahead(path);
        double L = Point.distance(currentPoint, currentLookahead);
        double radius = Math.pow(L, 2) / 2 * x;
        if (radius == 0) {
            return Double.POSITIVE_INFINITY;
        } else {
            return 1 / radius;
        }
    }

    /**
     * Calculates how far the robots tangent is from the lookahead point.
     * This method is used to check whether the robot is left or right of the point aswell.
     *
     * @param path current path
     * @return The X axis distance (relative to robot) from the lookahead, right side being positive.
     * @author orel
     * @author Paulo
     */
    private double distance_lookahead(Path path) {
        double robot_angle = Math.toRadians(drive.getAngle());
        double a = -Math.tan(robot_angle);
        double b = 1;
        double c = -Math.tan(robot_angle) * (currentPoint.getX() - currentPoint.getY());
        //Another point on the robots tangent
        double sign = Math.signum(Math.cos(robot_angle) * (currentLookahead.getX() - currentPoint.getX()) -
                Math.sin(robot_angle) * (currentLookahead.getY() - currentPoint.getY()));
        return sign * Math.abs(a * currentLookahead.getX() + b * currentLookahead.getY() + c) /
                Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }


    /**
     * calculates the speed needed in the right wheel and makes so we can apply it straight to the right engine
     *
     * @param path current path
     * @author lior
     * @return applied voltage to right engine
     */
    public double getRightSpeedVoltage(Path path) {
        double target_accel = Drivetrain.getRightSpeed()/0.02;
        return Constants.Kv*(closestPoint(path).getSpeed()*(2-curvatureCalculate(path)*Constants.TRACK_WIDTH)/2) + Constants.Ka*(target_accel) + Constants.Kp*(closestPoint(path).getSpeed()-Drivetrain.getRightSpeed);

    }

    /**
     * calculates the speed needed in the left wheel and makes so we can apply it straight to the left engine
     *
     * @param path current path
     * @author lior
     *@return applied voltage to left engine
     */
    public double getLeftSpeedVoltage(Path path) {
        double target_accel = Drivetrain.getLeftSpeed()/0.02;
        return Constants.Kv*(closestPoint(path).getSpeed()*(2+curvatureCalculate(path)*Constants.TRACK_WIDTH)/2) + Constants.Ka*(target_accel) + Constants.Kp*(closestPoint(path).getSpeed()-Drivetrain.getLeftSpeed;
    }

}
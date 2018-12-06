package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Drivetrain;
import robot.subsystems.drivetrain.pure_pursuit.*;
import robot.subsystems.drivetrain.pure_pursuit.Constants;

import static robot.Robot.drivetrain;

/**
 * The methods written here are all part of the Pure pursuit algorithm
 * all instances of the name 'the pure pursuit article' refer to this article by team DAWGMA 1712:
 * https://www.chiefdelphi.com/media/papers/download/5533
 */
public class PurePursue extends Command {
    private Drivetrain drive; //Instance of the subsystem
    private Path path; //Command specific path to follow
    private Point currentPoint; //holds X and Y variables for the robot
    private Point currentLookahead; //holds X and Y variables for the Lookahead point
    private int direction; //whether the robot drives forward or backwards (-1 or 1)
    private double lastLeftSpeed; //the last speed of the left encoder
    private double lastRightSpeed; //the last speed of the right encoder
    private double lastLeftDistance; //the last distance of the left encoder
    private double lastRightDistance; //the last distance of the right encoder
    private double lastLookaheadDistance; //distance of the last lookahead from the start of the path
    private double kP, kA, kV;
    private double lookaheadRadius;

    /**
     * An implementation of these command class. for more information see documentation on the wpilib command class.
     *
     * @param path            the Path class that the robot is going to follow
     * @param isReversed      states if the robot should drive forward or backwards along the path.
     * @param lookaheadRadius constant. the distance of the setpoint the robot tries to follow.
     * @param kP              driving constant. Multiplied by the difference between left and right speeds.
     * @param kA              driving constant. Multiplied by the robots acceleration.
     * @param kV              driving constant. Multiplied by the target velocity of the nearest point.
     */
    public PurePursue(Path path, boolean isReversed, double lookaheadRadius, double kP, double kA, double kV) {
        requires(drive);
        this.lookaheadRadius = lookaheadRadius;
        this.kP = kP;
        this.kA = kA;
        this.kV = kV;
        direction = isReversed ? -1 : 1;
        drive = drivetrain;
        this.path = path;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        currentPoint = new Waypoint(drivetrain.currentLocation.getX(), drivetrain.currentLocation.getY());
        lastLeftDistance = drivetrain.getLeftDistance();
        lastRightDistance = drivetrain.getRightDistance();
        currentLookahead = path.getWaypoint(0);
        lastLeftSpeed = direction * drive.getLeftSpeed();
        lastRightSpeed = direction * drive.getRightSpeed();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        updatePoint();
        updateLookaheadInPath(path);
        drive.setSpeed(getLeftSpeedVoltage(path), getRightSpeedVoltage(path));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (closestPoint(path) == path.getWaypoint(path.length() - 1) &&
                drive.getLeftSpeed() < Constants.STOP_SPEED_THRESH &&
                drive.getRightSpeed() < Constants.STOP_SPEED_THRESH);
    }

    // Called once after isFinished returns true
    protected void end() {
        drive.setSpeed(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

    /**
     * @author Paulo
     * Updates the Point object holding the robots x and y cooridinates
     */
    private void updatePoint() {
        //change in (change left encoder value + change in right encoder value)/2
        double distance = ((drivetrain.getLeftDistance() - lastLeftDistance) + (drivetrain.getRightDistance() - lastRightDistance)) / 2;

        //update the x, y coordinates based on the robot angle and the distance the robot moved.
        currentPoint.setX(currentPoint.getX() + direction * distance * Math.cos(drivetrain.getAngle() * (Math.PI / 180.0)));
        currentPoint.setY(currentPoint.getY() + direction * distance * Math.sin(drivetrain.getAngle() * (Math.PI / 180.0)));

        //updates values for next run
        lastLeftDistance = drivetrain.getLeftDistance();
        lastRightDistance = drivetrain.getRightDistance();
        drivetrain.currentLocation.setX(currentPoint.getX());
        drivetrain.currentLocation.setY(currentPoint.getY());
    }


    //Only Part of the function! doesn't run function through all of the path
    //https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899

    /**
     * This method finds the furthest point in a segment that is in a specified distance from the robot.
     * (Pure pursuit article, 'Following the path' > 'lookahead point', Page 10)
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
        Vector f = new Vector(point1, ref); //vector from the robot center to the start of the segment
        Vector p = new Vector(point2, point1); //vector of the segment

        /* using the equation: |t*p + f| = r, we try to find the value(s) of 't' where the length of the vector from the
         * robot center to the point is equal to the radius 'r'.
         * if you square both sides we get a quadratic formula with a, b, c. we use the quadratic formula and check for
         * the intersections, and then check if the intersection is within the segment (if t is between zero and one.)
         * the final method is: (p^2)*t^2 + (2*p*f)*t + (f^2 - r^2) = 0
         */

        double a = p.dot(p);
        double b = 2 * f.dot(p);
        double c = f.dot(f) - lookahead * lookahead;
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0)
            return null; //means that the circle doesn't reach the line
        else {
            discriminant = Math.sqrt(discriminant);
            double opt1 = (-b - discriminant) / (2 * a); //solve format of a quardatic formula
            double opt2 = (-b + discriminant) / (2 * a);
            if (opt1 >= 0 && opt1 <= 1) {
                return p.multiply(opt1).add(point1);
            }
            if (opt2 >= 0 && opt2 <= 1)
                return p.multiply(opt2).add(point1);
        }
        return null;
    }

    /**
     * Uses the 'FindNearPath' method on all segments to find the closest point.
     * Checks for the next intersection thats index is higher than the current lookahead point.
     *
     */
    private void updateLookaheadInPath(Path path) {
        for (int i = 0; i < path.length() - 1; i++) { //goes through each segment in path.
            Waypoint wp = findNearPath(currentPoint, lookaheadRadius, path.getWaypoint(i), path.getWaypoint(i + 1));
            if (wp != null) { //updates lookahead point to the first lookahead point the path finds
                if (Point.distance(wp, path.getWaypoint(i)) + path.getWaypoint(i).getDistance() > lastLookaheadDistance) {
                    lastLookaheadDistance = Point.distance(wp, path.getWaypoint(i)) + path.getWaypoint(i).getDistance();
                    currentLookahead = wp;
                    return;
                }
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
     * @return The curvature from the tangent of the robot to the setpoint
     * @author orel
     * @author Paulo
     */
    private double curvatureCalculate() {
        double x = distanceLookahead();
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
     * @return The X axis distance (relative to robot) from the lookahead, right side being positive.
     * @author orel
     * @author Paulo
     */
    private double distanceLookahead() {
        double robot_angle = Math.toRadians(drive.getAngle() + (direction == -1 ? 180 : 0));
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
     * Take the current output and gradually raise it to the target velocity, while making sure it doesn't change
     * in a rate that is faster than the maximum acceleration rate.
     *
     * @param input the target velocity.
     * @param lastOutput current output.
     * @param limitRate maximum acceleration rate
     * @return returns an updated output.
     * @author Paulo
     */
    public double limitRate(double input, double lastOutput, double limitRate) {
        double maxChange = 0.02 * limitRate;
        return lastOutput + Math.min(-maxChange, Math.max(input - lastOutput, maxChange));
    }

    /**
     * calculates the speed needed in the right wheel and makes so we can apply it straight to the right engine
     *
     * @param path current path
     * @return applied voltage to right engine
     * @author lior
     */
    public double getRightSpeedVoltage(Path path) {
        double target_accel = (drive.getRightSpeed() - lastRightSpeed) / 0.02;
        lastRightSpeed = drive.getRightSpeed();
        return kV * (closestPoint(path).getSpeed() * (2 - curvatureCalculate() * Constants.ROBOT_WIDTH) / 2) +
                kA * (target_accel) +
                kP * (closestPoint(path).getSpeed() - drive.getRightSpeed());

    }

    /**
     * calculates the speed needed in the left wheel and makes so we can apply it straight to the left engine
     *
     * @param path current path
     * @return applied voltage to left engine
     * @author lior
     */
    public double getLeftSpeedVoltage(Path path) {
        double target_accel = (drive.getLeftSpeed() - lastLeftSpeed) / 0.02;
        lastLeftSpeed = drive.getLeftSpeed();
        return kV * (closestPoint(path).getSpeed() * (2 + curvatureCalculate() * Constants.ROBOT_WIDTH) / 2) +
                kA * (target_accel) +
                kP * (closestPoint(path).getSpeed() - drive.getLeftSpeed());
    }

}

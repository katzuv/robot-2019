package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static robot.Robot.drivetrain;

/**
 *
 */
public class PurePursue extends Command {
    private Path path; //Command specific path to follow
    private Point currentPoint; //holds X and Y variables for the robot
    private Point currentLookahead; //holds X and Y variables for the Lookahead point
    private int direction; //whether the robot drives forward or backwards (-1 or 1)
    private double lastLeftSpeed; //the last speed of the left encoder
    private double lastRightSpeed; //the last speed of the right encoder
    private double lastLeftEncoder; //the last distance of the left encoder
    private double lastRightEncoder; //the last distance of the right encoder
    //private double initAngle;
    private double lastLookaheadDistance; //distance of the last lookahead from the start of the path
    private double kP, kA, kV;
    private double lookaheadRadius;

    /**
     * A command class.
     *
     * @param path            the Path class that the robot is going to follow
     * @param isReversed      states if the robot should drivetrain forward or backwards along the path.
     * @param lookaheadRadius constant. the distance of the setpoint the robot tries to follow.
     * @param kP              driving constant. Multiplied by the difference between left and right speeds.
     * @param kA              driving constant. Multiplied by the robots acceleration.
     * @param kV              driving constant. Multiplied by the target velocity of the nearest point.
     */
    public PurePursue(Path path, boolean isReversed, double lookaheadRadius, double kP, double kA, double kV) {
        requires(drivetrain);
        this.lookaheadRadius = lookaheadRadius;
        this.kP = kP;
        this.kA = kA;
        this.kV = kV;
        direction = isReversed ? -1 : 1;
        this.path = path;
        // Use requires() here to declare subsystem dependencies
        // eg.
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        currentPoint = new Waypoint(drivetrain.currentLocation.getX(), drivetrain.currentLocation.getY());
        lastLeftEncoder = drivetrain.getLeftDistance();
        lastRightEncoder = drivetrain.getRightDistance();
        //initAngle = drivetrain.getAngle() + (direction == -1 ? 180 : 0);
        currentLookahead = path.getWaypoint(0);
        lastLeftSpeed = direction * drivetrain.getLeftSpeed();
        lastRightSpeed = direction * drivetrain.getRightSpeed();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        updatePoint();
        updateLookaheadInPath(path);
        SmartDashboard.putNumber("voltagesent sent left" , getLeftSpeedVoltage(path));
        SmartDashboard.putNumber("voltage sent right" , getRightSpeedVoltage(path));
        SmartDashboard.putNumber("curvature calculate" , curvatureCalculate());
        drivetrain.setSpeed(getLeftSpeedVoltage(path), getRightSpeedVoltage(path));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (closestPoint(path) == path.getWaypoint(path.length() - 1) &&
                drivetrain.getLeftSpeed() < Constants.STOP_SPEED_THRESH &&
                drivetrain.getRightSpeed() < Constants.STOP_SPEED_THRESH);
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
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
        double distance = -((drivetrain.getLeftDistance() - lastLeftEncoder) + (drivetrain.getRightDistance() - lastRightEncoder)) / 2;

        currentPoint.setX(currentPoint.getX() + direction * distance * Math.cos(drivetrain.getAngle() * (Math.PI / 180.0)));
        currentPoint.setY(currentPoint.getY() + direction * distance * Math.sin(drivetrain.getAngle() * (Math.PI / 180.0)));

        lastLeftEncoder = drivetrain.getLeftDistance();
        lastRightEncoder = drivetrain.getRightDistance();
        drivetrain.currentLocation.setX(currentPoint.getX());
        drivetrain.currentLocation.setY(currentPoint.getY());
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
        assert ref != null;
        assert point1 != null;
        assert point2 != null;
        Vector p = new Vector(point1, point2);
        Vector f = new Vector(ref, point1);

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
                return p.multiply(opt1).add(point1);
            }
            if (opt2 >= 0 && opt2 <= 1)
                return p.multiply(opt2).add(point1);
            return null;
        }
    }

    /**
     * Uses the 'FindNearPath' method on all segments to find the closest point.
     * Checks for the next intersection thats index is higher than the current lookahead point.
     *
     * @return the Lookahead Point.
     * @path the path the robot is driving on.
     */
    private void updateLookaheadInPath(Path path) {
        for (int i = 0; i < path.length() - 1; i++) {
            Waypoint wp = findNearPath(currentPoint, lookaheadRadius, path.getWaypoint(i), path.getWaypoint(i + 1));
            if (wp != null) {
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
        double radius = Math.pow(L, 2);

        if (radius == 0) {
            return Math.pow(10,5);
        } else {
            return (x*2) / radius;
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
        double robot_angle = Math.toRadians(drivetrain.getAngle() + (direction == -1 ? 180 : 0));
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
     * @return applied voltage to right engine
     * @author lior
     */
    public double getRightSpeedVoltage(Path path) {
        double target_accel = (drivetrain.getRightSpeed() - lastRightSpeed) / 0.02;
        lastRightSpeed = drivetrain.getRightSpeed();
        if (curvatureCalculate() >= Math.pow(10,3))
            return kV*(closestPoint(path).getSpeed());
        else
        return kV * (closestPoint(path).getSpeed() * (2 - curvatureCalculate() * Constants.TRACK_WIDTH) / 2) +
                kA * (target_accel) +
                kP * (closestPoint(path).getSpeed() - drivetrain.getRightSpeed());

    }

    /**
     * calculates the speed needed in the left wheel and makes so we can apply it straight to the left engine
     *
     * @param path current path
     * @return applied voltage to left engine
     * @author lior
     */
    public double getLeftSpeedVoltage(Path path) {
        double target_accel = (drivetrain.getLeftSpeed() - lastLeftSpeed) / 0.02;
        lastLeftSpeed = drivetrain.getLeftSpeed();
        return kV * (closestPoint(path).getSpeed() * (2 + curvatureCalculate() * Constants.TRACK_WIDTH) / 2) +
                kA * (target_accel) +
                kP * (closestPoint(path).getSpeed() - drivetrain.getLeftSpeed());
    }

}
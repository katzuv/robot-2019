package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.utilities.Point;
import robot.utilities.Vector;

import static robot.Robot.drivetrain;

/**
 * This driving algorithm is a remaster of our 2018 driving algorithm.
 * Most of the changes here are simply readability, and a few upgrades with the work that we have done
 * throughout 2019.
 *
 * @author paulo
 */
public class VectorPursuit extends Command {
    public int direction; //whether the robot drives forward or backwards (-1 or 1)
    private Path path; //Command specific path to follow
    private Point currentPoint = new Point(0, 0); //holds X and Y variables for the robot
    private double lastLeftSpeed; //the last speed of the left encoder
    private double lastRightSpeed; //the last speed of the right encoder
    private double lastLeftEncoder; //the last distance of the left encoder
    private double lastRightEncoder; //the last distance of the right encoder
    private double lastLookaheadDistance; //distance of the last lookahead from the start of the path
    private double kP, kA, kV;
    private double lookaheadRadius;
    private boolean isRelative;
    private double initAngle;
    private double delta;
    private double lastTimestamp;

    /**
     * An implementation of these command class. for more information see documentation on the wpilib command class.
     *
     * @param path            the Path class that the robot is going to follow
     * @param isReversed      states if the robot should drivetrain forward or backwards along the path.
     * @param lookaheadRadius constant. the distance of the setpoint the robot tries to follow.
     * @param kP              driving constant. Multiplied by the difference between left and right speeds.
     * @param kA              driving constant. Multiplied by the robots acceleration.
     * @param kV              driving constant. Multiplied by the target velocity of the nearest point.
     */
    public VectorPursuit(Path path, double lookaheadRadius, double kP, double kA, double kV, boolean isRelative, boolean isReversed) {
        requires(drivetrain);
        this.lookaheadRadius = lookaheadRadius;
        this.kP = kP;
        this.kA = kA;
        this.kV = kV;
        direction = isReversed ? -1 : 1;
        this.path = path;

        this.isRelative = isRelative;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (isRelative) {
            initAngle = drivetrain.getAngle() + (direction == -1 ? 180 : 0);
            currentPoint = new Point(0, 0);
        } else {
            initAngle = 0;
            currentPoint = new Point(drivetrain.currentLocation.getX(), drivetrain.currentLocation.getY());
        }

        lastLeftEncoder = drivetrain.getLeftDistance() * direction;
        lastRightEncoder = drivetrain.getRightDistance() * direction;
        lastLeftSpeed = direction * drivetrain.getLeftSpeed();
        lastRightSpeed = direction * drivetrain.getRightSpeed();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (!isFinished()) {
            updatePoint();
            drivetrain.setVelocity(getLeftSpeedVoltage(path) * direction, getRightSpeedVoltage(null) * direction);
            SmartDashboard.putNumber("x", drivetrain.currentLocation.getX());
            SmartDashboard.putNumber("y", drivetrain.currentLocation.getY());
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        SmartDashboard.putString("test: Final point?", path.getWaypoint(path.length() - 2).toString());
        boolean closeToLast = (Math.abs(drivetrain.currentLocation.getX() - path.getWaypoint(path.length() - 1).getX()) <= 0.03 &&
                Math.abs(drivetrain.currentLocation.getY() - path.getWaypoint(path.length() - 2).getY()) <= 0.03);
        return (closeToLast &&
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
    }

    /**
     * @author Paulo
     * Updates the Point object holding the robots x and y cooridinates
     */
    private void updatePoint() {
        //change in (change left encoder value + change in right encoder value)/2
        double distance = ((drivetrain.getLeftDistance() * direction - lastLeftEncoder) + (drivetrain.getRightDistance() * direction - lastRightEncoder)) / 2;

        //update the x, y coordinates based on the robot angle and the distance the robot moved.
        currentPoint.setX(currentPoint.getX() + direction * distance * Math.sin((drivetrain.getAngle() - initAngle) * (Math.PI / 180.0)));
        currentPoint.setY(currentPoint.getY() + direction * distance * Math.cos((drivetrain.getAngle() - initAngle) * (Math.PI / 180.0)));

        //updates values for next run
        lastLeftEncoder = drivetrain.getLeftDistance() * direction;
        lastRightEncoder = drivetrain.getRightDistance() * direction;
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
        Vector p = new Vector(point1, point2);
        Vector f = new Vector(ref, point1);

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
        SmartDashboard.putNumber("target", closest.getSpeed());
        return closest;
    }

    /**
     * calculates the speed needed in the right wheel and makes so we can apply it straight to the right engine
     *
     * @param movementVector current path
     * @return applied voltage to right engine
     * @author lior
     */
    public double getRightSpeedVoltage(Vector movementVector) {
        //double leftVelocity = movementVector.magnitude() * Math.cos(movementVector.angle()-drivetrain.getAngle()) - movementVector.magnitude() * Math.sin(movementVector.angle() - drivetrain.getAngle()) * r / h;
        return 0;
    }

    /**
     * calculates the speed needed in the left wheel and makes so we can apply it straight to the left engine
     *
     * @param path current path
     * @return applied voltage to left engine
     * @author lior
     */
    public double getLeftSpeedVoltage(Path path) {
        SmartDashboard.putString("~Vector", getVelocity().toString());
        SmartDashboard.putString("~Forward", driveVelocityVector(path).toString());
        SmartDashboard.putString("~Distance", getDistanceVectorFromPath(path).toString());
        SmartDashboard.putString("~Robot location", currentPoint.toString());
        SmartDashboard.putNumber("~Distance from path", getDistanceFromPath(path));
        return 0;
    }

    public double getLeftVelocity() {
        return 0;
    }

    /**
     * call the timestamp on the robot
     *
     * @return timestamp in seconds
     */
    private double getTimeDelta() {
        return this.delta;
    }

    private void updateTimeDelta() {
        delta = Math.max(Timer.getFPGATimestamp() - lastTimestamp, 0.005);
        lastTimestamp += delta;
    }

    /**
     * Calculate the velocity needed to get to a target velocity, at an x distance away
     *
     * @param targetSpeed
     * @param acceleration
     * @param startPos
     * @param targetPos
     * @return
     */
    public static double velocityByDistance(double targetSpeed, double acceleration, double startPos, double targetPos) {
        return Math.sqrt(targetSpeed * targetSpeed + 2 * Math.abs(acceleration) * Math.abs(targetPos - startPos));
    }

    /**
     * get the vector of movement based on the path
     *
     * @return
     */
    private Vector getVelocity() {
        Vector errorVelocityVector = getDistanceVectorFromPath(path).normalize().multiply(velocityByDistance(0, Constants.errorAcceleration, 0, getDistanceVectorFromPath(path).magnitude()));
        Vector driveVelocityVector = driveVelocityVector(path);
        if(errorVelocityVector.magnitude()>0) {
            driveVelocityVector.normalize().multiply(Math.max(driveVelocityVector.magnitude(),
                    Math.pow(Constants.MAX_VELOCITY, 2) - Math.pow(errorVelocityVector.magnitude(), 2)
            )); //make sure the forward velocity doesn't pass the maximal forward velocity

            return driveVelocityVector.add(errorVelocityVector);
        }
        return driveVelocityVector;
    }


    /**
     * Deprecated.
     * @param path
     * @return
     */
    private double getDistanceFromPath(Path path) {
        //there are two possible segments now, so we have to try the lowest distance between the two of them
        double closestDistance = 100000000;
        Waypoint closestPoint;
        for (int i = 0; i < path.length() - 1; i++) {
            if (Math.abs(distBetweenPointAndLine(path.getWaypoint(i), path.getWaypoint(i + 1), currentPoint)) < Math.abs(closestDistance)) {
                closestDistance = distBetweenPointAndLine(path.getWaypoint(i), path.getWaypoint(i + 1), currentPoint);
            }
        }
        return closestDistance;
    }

    /**
     * gets the distance between a point and a segment.
     * if the point on the right of the segment, the number will be positive (right is checked by the start and end edge)
     * @param edge1 starting edge of the segment
     * @param edge2 final edge of the segment
     * @param target reference point, calculating the distance
     * @return distance from the segment in meters.
     */
    public double distBetweenPointAndLine(Point edge1, Point edge2, Point target) {
        Vector firstVector = new Vector(edge1, edge2); //TODO: might not work
        Vector secondVector = new Vector(edge1, target);

        double a = Math.signum(firstVector.angle() - secondVector.angle());
        return a * Point.distance(target, getPointOnSegment(edge1, edge2, target)); //TODO: needs testing
    }

    /**
     * find the point on a line which is closest to a point.
     * @param edge1 first edge of the segment
     * @param edge2 second edge of the segment
     * @param p point of reference
     * @return the point which creates with p a perpendicular line to the segment
     */
    public Point getPointOnSegment(Point edge1, Point edge2, Point p) {
        //define the segment as a linear equation: y = m * x + n
        if (edge1.getX() == edge2.getX())
            return new Point(edge1.getX(), p.getY());
        double slope = (edge2.getY() - edge1.getY()) / (edge2.getX() - edge1.getX());
        double n = edge1.getY() - slope * edge1.getX();
        if (slope == 0)
            return new Point(p.getX(), n); //if the slope is equal to 0, calculate the point to prevent errors.

        /*
         * we want to find the point that is on the segment.
         * if we take the following equation: y = m * x + n
         * y - n = m * x...
         */
        double x = (p.getY() - n + (p.getX() / slope)) / (slope + 1 / slope); // TODO: ill explain this i promise
        double y = slope * x + n;
        return new Point(x, y);
    }

    /**
     * finds the nearest point on the path, and gets the velocity from the waypoints at the location
     * @param path
     * @return the velocity of the path, in the direction of the path in m/s
     */
    public Vector driveVelocityVector(Path path) {//TODO: not right at all.
        double closestDistance = 100000000;
        int closestPointIndex = 0;
        for (int i = 0; i < path.length() - 1; i++) {
            if (Math.abs(distBetweenPointAndLine(path.getWaypoint(i), path.getWaypoint(i + 1), currentPoint)) < Math.abs(closestDistance)) {
                closestDistance = distBetweenPointAndLine(path.getWaypoint(i), path.getWaypoint(i + 1), currentPoint);
                closestPointIndex = i;
            }
        }
        //Create a vector between both segments, which has a magnitude of the path velocity TODO:get the correct value between two waypoints
        return new Vector(path.getWaypoint(closestPointIndex), path.getWaypoint(closestPointIndex + 1)).normalize().multiply(path.getWaypoint(closestPointIndex + 1).getSpeed());
    }

    /**
     * returns the distance from the path as a vector, parallel to the segment
     * @param path
     * @return vector, facing the segment
     */
    private Vector getDistanceVectorFromPath(Path path) {
        //there are two possible segments now, so we have to try the lowest distance between the two of them
        return new Vector(currentPoint, getPointOnSegment(getClosestSegment(path, currentPoint)[0],getClosestSegment(path, currentPoint)[1], currentPoint));
    }

    /**
     * find the nearest segment to a point from a path
     *
     * @param path Path class
     * @param p a Point class of the target point
     * @return both edges of the segment
     */
    private Waypoint[] getClosestSegment(Path path, Point p){ //TODO: should ignore points already passed, using distanceOnPath
        double closestDistance = 100000000;
        int closestPointIndex = 0;
        //Finds the distance between the point and the center of each segment, and finds the smallest
        for (int i = 0; i < path.length() - 1; i++) {
            if(Point.distance(p,Point.average(path.getWaypoint(i),path.getWaypoint(i+1))) <= closestDistance){
                closestDistance = distBetweenPointAndLine(path.getWaypoint(i), path.getWaypoint(i + 1), currentPoint);
                closestPointIndex = i;
            }
        }
        return new Waypoint[] {path.getWaypoint(closestPointIndex), path.getWaypoint(closestPointIndex + 1)};
    }
}

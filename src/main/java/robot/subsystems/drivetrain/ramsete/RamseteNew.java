package robot.subsystems.drivetrain.ramsete;

import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.Velocity;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

public class RamseteNew {

    // Should be greater than zero and this increases correction
    private double kBeta = Constants.kBeta; //1.5;

    // Should be between zero and one and this increases dampening
    private double kZeta = Constants.kZeta; //0.7;

    // Holds what segment we are on
    private int WaypointIndex;
    private Waypoint currentWaypoint;

    // The trajectory to follow
    private Path path;

    // The robot's x and y position and angle

    // Variable used to calculate linear and angular velocity
    private double lastTheta, nextTheta;
    private double k, thetaError, sinThetaErrorOverThetaError;
    private double desiredAngularVelocity, linearVelocity, angularVelocity;
    private double odometryError;

    // Constants
    private static final double EPSILON = 0.00000001;

    // Variable for holding velocity for robot to drive on
    private Velocity velocity;
    private double left, right;


    public RamseteNew(Path path)
    {
        //ternary operator, if direction is forward return trajectory, else return the reversed path
        this.path = path;//direction == direction ? trajectory : TrajectoryUtil.reversePath(trajectory) check if needed

        WaypointIndex = 0;



    }

    public RamseteNew(Path path, double b, double zeta)
    {
        this(path);

        this.kBeta = b;
        this.kZeta = zeta;
    }

    public Velocity getVelocity()
    {
        if (isFinished())
        {
            return new Velocity(0, 0);
        }

        currentWaypoint = path.getWaypoint(WaypointIndex);

        desiredAngularVelocity = calculateDesiredAngular();

        linearVelocity = calculateLinearVelocity(currentWaypoint.getX(), currentWaypoint.getY(), currentWaypoint.getHeading(), currentWaypoint.getSpeed(), desiredAngularVelocity);

        angularVelocity = calculateAngularVelocity(currentWaypoint.getX(), currentWaypoint.getY(), currentWaypoint.getSpeed(), currentWaypoint.getSpeed(), desiredAngularVelocity);

        return new Velocity(linearVelocity, angularVelocity);
    }

    public void getNextVelocities()
    {
        velocity = getVelocity();

        left = (-(velocity.getAngular() * Constants.ROBOT_WIDTH) + (2 * velocity.getLinear())) / 2;
        right = ((velocity.getAngular() * Constants.ROBOT_WIDTH) + (2 * velocity.getLinear())) / 2;

        Robot.drivetrain.setLeftVelocity(Robot.drivetrain.convertDistanceToTicks(left) / 10);
        Robot.drivetrain.setRightVelocity(Robot.drivetrain.convertDistanceToTicks(right) / 10);

        WaypointIndex++;


    }

    private double calculateDesiredAngular()
    {
        if (WaypointIndex < path.length() - 1)
        {
            lastTheta = path.getWaypoint(WaypointIndex).getHeading();
            nextTheta = path.getWaypoint(WaypointIndex + 1).getHeading();
            return boundHalfRadians(nextTheta - lastTheta) / currentWaypoint.getDistance();
        }
        else
        {
            return 0;
        }
    }

    private double calculateLinearVelocity(double desiredX, double desiredY, double desiredTheta,
                                           double desiredLinearVelocity, double desiredAngularVelocity)
    {
        // System.out.println("DesiredLinearVelocity: " + desiredLinearVelocity);
        System.out.println("Desired X: " + desiredX);

        // System.out.println("thetaError: " + thetaError);
        k = calculateK(desiredLinearVelocity, desiredAngularVelocity);

        thetaError = boundHalfRadians(desiredTheta - Math.toRadians(Robot.drivetrain.getAngle()));

        odometryError = (Math.cos(Math.toRadians(Robot.drivetrain.getAngle())) * (desiredX - Robot.drivetrain.getCurrentLocation().getX()))
                + (Math.sin(Math.toRadians(Robot.drivetrain.getAngle()) * (desiredY - Robot.drivetrain.getCurrentLocation().getY())));

        System.out.println("k * odometryError: " + (k * odometryError));

        return (desiredLinearVelocity * Math.cos(thetaError)) + (k * odometryError);
    }

    private double calculateAngularVelocity(double desiredX, double desiredY, double desiredTheta,
                                            double desiredLinearVelocity, double desiredAngularVelocity)
    {
        k = calculateK(desiredLinearVelocity, desiredAngularVelocity);

        thetaError = boundHalfRadians(desiredTheta - Math.toRadians(Robot.drivetrain.getAngle()));

        if (Math.abs(thetaError) < EPSILON)
        {
            // This is for the limit as sin(x)/x approaches zero
            sinThetaErrorOverThetaError = 1;
        }
        else
        {
            sinThetaErrorOverThetaError = Math.sin(thetaError) / thetaError;
        }

        odometryError = (Math.cos(Math.toRadians(Robot.drivetrain.getAngle()) * (desiredY - Robot.drivetrain.getCurrentLocation().getY()))
                - (Math.sin(Math.toRadians(Robot.drivetrain.getAngle()) * (desiredX - Robot.drivetrain.getCurrentLocation().getX()))));

        return desiredAngularVelocity + (kBeta * desiredLinearVelocity * sinThetaErrorOverThetaError * odometryError)
                + (k * thetaError);
    }

    private double calculateK(double desiredLinearVelocity, double desiredAngularVelocity)
    {
        return 2 * kZeta * Math.sqrt(Math.pow(desiredAngularVelocity, 2) + (kBeta * Math.pow(desiredLinearVelocity, 2)));
    }

    private double boundHalfRadians(double radians)
    {
        while (radians >= Math.PI)
            radians -= Constants.TWO_PI;
        while (radians < -Math.PI)
            radians += Constants.TWO_PI;
        return radians;
    }

    public Waypoint currenntWaypoint()
    {
        return currentWaypoint;
    }

    public boolean isFinished()
    {
        return WaypointIndex >= path.length();
    }


    public int getWaypointIndex()
    {
        return this.WaypointIndex;
    }

    public void printCurrentLocation() {
        System.out.printf("position" + Robot.drivetrain.currentLocation);
        System.out.println("Actual encoder left: " + Robot.drivetrain.getLeftDistance());
        System.out.println("Actual encoder right: " + Robot.drivetrain.getRightDistance());
    }
}

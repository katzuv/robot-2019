package robot.subsystems.drivetrain.ramsete;

import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.Velocity;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

import static robot.subsystems.drivetrain.Constants.kBeta;
import static robot.subsystems.drivetrain.Constants.kZeta;

public class RamseteNew {


    // the Waypoints
    private int WaypointIndex;
    private Waypoint currentWaypoint;

    // The path
    private Path path;

    private double lastTheta, nextTheta;
    private double k, thetaError, sinThetaError;
    private double desiredAngularVelocity, linearVelocity, angularVelocity, leftVelocity, rightVelocity, error;
    private Velocity velocity;


    public RamseteNew(Path path)
    {
        //ternary operator, if direction is forward return trajectory, else return the reversed path
        this.path = path;//direction == direction ? trajectory : TrajectoryUtil.reversePath(trajectory) check if needed

        WaypointIndex = 0;



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

        leftVelocity = (-(velocity.getAngular() * Constants.ROBOT_WIDTH) + (2 * velocity.getLinear())) / 2;
        rightVelocity = ((velocity.getAngular() * Constants.ROBOT_WIDTH) + (2 * velocity.getLinear())) / 2;

        Robot.drivetrain.setLeftVelocity(Robot.drivetrain.convertLeftDistanceToTicks(leftVelocity) / 10);
        Robot.drivetrain.setRightVelocity(Robot.drivetrain.convertRightDistanceToTicks(rightVelocity) / 10);

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

        error = (Math.cos(Math.toRadians(Robot.drivetrain.getAngle())) * (desiredX - Robot.drivetrain.currentLocation.getX()))
                + (Math.sin(Math.toRadians(Robot.drivetrain.getAngle()) * (desiredY - Robot.drivetrain.currentLocation.getY())));


        return (desiredLinearVelocity * Math.cos(thetaError)) + (k * error);
    }

    private double calculateAngularVelocity(double desiredX, double desiredY, double desiredTheta,
                                            double desiredLinearVelocity, double desiredAngularVelocity)
    {
        k = calculateK(desiredLinearVelocity, desiredAngularVelocity);

        thetaError = boundHalfRadians(desiredTheta - Math.toRadians(Robot.drivetrain.getAngle()));

        if (Math.abs(thetaError) < Constants.MIN_ERROR)
        {
            // This is for the limit as sin(x)/x approaches zero
            sinThetaError = 1;
        }
        else
        {
            sinThetaError = Math.sin(thetaError) / thetaError;
        }

        error = (Math.cos(Math.toRadians(Robot.drivetrain.getAngle()) * (desiredY - Robot.drivetrain.currentLocation.getY()))
                - (Math.sin(Math.toRadians(Robot.drivetrain.getAngle()) * (desiredX - Robot.drivetrain.currentLocation.getX()))));

        return desiredAngularVelocity + (kBeta * desiredLinearVelocity * sinThetaError * error)
                + (k * thetaError);
    }

    private double calculateK(double desiredLinearVelocity, double desiredAngularVelocity)
    {
        return 2 * kZeta * Math.sqrt(Math.pow(desiredAngularVelocity, 2) + (kBeta * Math.pow(desiredLinearVelocity, 2)));
    }

    private double boundHalfRadians(double radians)
    {
        while (radians >= Math.PI)
            radians -= 2*Math.PI;
        while (radians < -Math.PI)
            radians += 2*Math.PI;
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

    public void printLocation() {
        System.out.printf("position" + Robot.drivetrain.currentLocation);
        System.out.println("Actual encoder left: " + Robot.drivetrain.getLeftDistance());
        System.out.println("Actual encoder right: " + Robot.drivetrain.getRightDistance());
    }
}

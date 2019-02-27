package robot.subsystems.drivetrain.ramsete;

import org.ghrobotics.lib.mathematics.MathExtensionsKt;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class RamseteTracker {
    private final double kBeta;
    private final double kZeta;

    protected double[] calculateState(Waypoint waypoint, Waypoint robot, Waypoint nextPoint) {

        double errorX = waypoint.getX() - robot.getX();
        double errorY = waypoint.getY() - robot.getY();

        double wd = waypoint.getSpeed() * waypoint.getCurvature();

        double k1 = 2.0 * this.kZeta * Math.sqrt(wd * wd + this.kBeta * waypoint.getSpeed() * waypoint.getSpeed());

        double angleError = Math.atan2(nextPoint.getY() - waypoint.getY(), nextPoint.getX() - waypoint.getX());
        angleError = 0;
        double linearVelocity = waypoint.getSpeed() * Math.cos(angleError) + k1 * errorX;
        double angularVelocity = wd + this.kBeta * waypoint.getSpeed() * sinc(angleError) * errorY + k1 * angleError;
        angularVelocity = -angularVelocity;
        double[] speeds = new double[2];
        speeds[0] = linearVelocity - (angularVelocity * Constants.ROBOT_WIDTH / 2);
        speeds[1] = linearVelocity + (angularVelocity * Constants.ROBOT_WIDTH / 2);
        return speeds;
    }

    public RamseteTracker(double kBeta, double kZeta) {
        this.kBeta = kBeta;
        this.kZeta = kZeta;
    }

    private double sinc(double theta) {
        return MathExtensionsKt.epsilonEquals(theta, 0.0D) ? 1.0D - 1.0D / 6.0D * theta * theta : Math.sin(theta) / theta;
    }

    public static List<Waypoint> waypoints() {
        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(0, 0, 0.1524, 0.2, 0));
        waypoints.add(new Waypoint(0, 0.3, 0.3048, 0.4, 0));
        waypoints.add(new Waypoint(0, 0.4572, 0.4572, 0.7, 0));
        waypoints.add(new Waypoint(0, 0.6096, 0.6096, 1, 0));
        waypoints.add(new Waypoint(0, 0.8096, 0.8096, 0.6, 0));
        waypoints.add(new Waypoint(0, 0.91, 0.91, 0, 0));
        return waypoints;
    }

    public static void main(String[] args) {
//        Path path = new Path(new Waypoint[]{new Waypoint(0, 0), new Waypoint(0, 1)});
//        path.generateAll(robot.subsystems.drivetrain.pure_pursuit.Constants.WEIGHT_DATA, robot.subsystems.drivetrain.pure_pursuit.Constants.WEIGHT_SMOOTH, robot.subsystems.drivetrain.pure_pursuit.Constants.TOLERANCE, robot.subsystems.drivetrain.pure_pursuit.Constants.MAX_ACCEL, robot.subsystems.drivetrain.pure_pursuit.Constants.MAX_PATH_VELOCITY);
//        System.out.println(path);
//        RamseteTracker tracker = new RamseteTracker(2, 0.7);
//        System.out.println(path);
//        for (int i = 0; i < path.length(); i++) {
//            Waypoint nextPoint;
//            if (i < path.length() - 1) {
//                nextPoint = path.getWaypoint(i + 1);
//            } else {
//                nextPoint = new Waypoint(0, 0, 0, 0, 0);
//            }
//            System.out.println(tracker.calculateState(path.getWaypoint(i), new Waypoint(0, 0), nextPoint));
//        }
        RamseteTracker tracker = new RamseteTracker(2, 0.7);
        for (int i = 0; i < waypoints().size(); i++) {
            Waypoint nextPoint;
            if (i < waypoints().size() - 1) {
                nextPoint = waypoints().get(i + 1);
            } else {
                nextPoint = new Waypoint(0, 0, 0, 0, 0);
            }
            System.out.println(Arrays.toString(tracker.calculateState(waypoints().get(i), new Waypoint(0, 0), nextPoint)));
        }
    }

}

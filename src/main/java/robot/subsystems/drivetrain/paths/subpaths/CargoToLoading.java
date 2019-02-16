package robot.subsystems.drivetrain.paths.subpaths;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.*;

/**
 *
 */
public class CargoToLoading extends Command {

    NetworkTableEntry distanceEntry;
    NetworkTableEntry angleEntry;

    public CargoToLoading() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }


    // Called just before this Command runs the first time
    protected void initialize() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("vision");
        angleEntry = table.getEntry("angle");
        distanceEntry = table.getEntry("distance");

        Path path = new Path(new Waypoint[]{new Waypoint(-2, -0.75), new Waypoint(-1.7, -3.5)});

        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
        pursue.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        Waypoint targetWP = target(angleEntry.getDouble(0), distanceEntry.getDouble(0));
        if (angleEntry.getDouble(0) != 0 && distanceEntry.getDouble(0) != 0) {
            if (Point.distance(Robot.drivetrain.currentLocation, targetWP) >= Constants.MIN_DISTANCE) {
                Path path = generateFromVision(angleEntry.getDouble(0), distanceEntry.getDouble(0));

                PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
                pursue.start();
            }

        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Point.distance(Robot.drivetrain.currentLocation, target(angleEntry.getDouble(0), distanceEntry.getDouble(0)))
                <= 0;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    /**
     * @param angle    from target
     * @param distance from target
     * @return the path
     */
    private Path generateFromVision(double angle, double distance) {
        double targetDistance = distance;
        Waypoint target = target(angle, targetDistance / 100);
        Waypoint middleWP = getMiddleWP(target);
        Path path1 = new Path(new Waypoint[]{new Waypoint(0, 0), middleWP, target});
        path1.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        System.out.println(path1);
        return path1;
    }

    private Waypoint getMiddleWP(Waypoint target) {
        return new Waypoint(0, target.getY() - target.getY() / 2);
    }

    private Waypoint target(double angle, double distance) {
        return new Waypoint(Math.sin(Math.toRadians(angle)) * distance + 0.15, Math.cos(Math.toRadians(angle)) * distance);
    }
}
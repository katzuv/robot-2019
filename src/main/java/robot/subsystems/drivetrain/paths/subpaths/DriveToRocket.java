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
public class DriveToRocket extends Command {

    NetworkTableEntry distanceEntry;
    NetworkTableEntry angleEntry;

    public DriveToRocket() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("vision");
        angleEntry = table.getEntry("angle");
        distanceEntry = table.getEntry("distance");

        Path path = new Path(new Waypoint[]{new Waypoint(0, 3.5), new Waypoint(2, 3.5)});
        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
        pursue.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (angleEntry.getDouble(0) != 0 && distanceEntry.getDouble(0) != 0) {
            Waypoint targetWP =target(angleEntry.getDouble(0), distanceEntry.getDouble(0));
            Waypoint middle =getMiddleWP(targetWP);
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

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }


    /**
     * @param angle    from target
     * @param distance from target
     * @return full path to target
     */
    private Path generateFromVision(double angle, double distance) {
        Waypoint target = target(angle, distance / 100/*centimeters to meters*/);
        Waypoint middleWP = getMiddleWP(target);
        Path path1 = new Path(new Waypoint[]{new Waypoint(0, 0), middleWP, target});
        System.out.println(path1);
        path1.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        return path1;
    }

    /**
     * @param target Waypoint
     * @return the middle way point
     */
    private Waypoint getMiddleWP(Waypoint target) {
        return new Waypoint(0, target.getY() - target.getY() / 2);
    }

    /**
     *
     * @param angle from target
     * @param distance from target
     * @return the target way point
     */
    private Waypoint target(double angle, double distance) {
        return new Waypoint(Math.sin(Math.toRadians(angle)) * distance + 0.15, Math.cos(Math.toRadians(angle)) * distance);
    }
}
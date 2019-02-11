package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.*;

/**
 *
 */
public class GamePiecePickup extends Command {

    public GamePiecePickup() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    NetworkTableEntry targetAngleEntry;
    NetworkTableEntry targetDistanceEntry;
    private double currentDistanceUsed;


    /**
     * take distance and angle from the network tables than generate th pure pursue by that parameters
     */
    // Called just before this Command runs the first time
    protected void initialize() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("vision");
        targetAngleEntry = table.getEntry("angle");
        targetDistanceEntry = table.getEntry("distance");
        currentDistanceUsed = targetDistanceEntry.getDouble(0);
        Path path1 = generateFromVision(targetAngleEntry.getDouble(0), targetDistanceEntry.getDouble(0));
        path1.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        SmartDashboard.putNumber("target distance", targetDistanceEntry.getDouble(0) / 100);
        SmartDashboard.putNumber("target angle ", targetAngleEntry.getDouble(0));
        PurePursue pursue = new PurePursue(path1, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
        pursue.start();
        System.out.println(path1);
    }

    /**
     * when the robot reach the middleWP the robot regenerate the purePursuit unless the distance is minimal
     */
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Waypoint targetWP =target(targetAngleEntry.getDouble(0), targetDistanceEntry.getDouble(0));
        Waypoint middle =getMiddleWP(targetWP);
        Path path = generateFromVision(targetAngleEntry.getDouble(0), targetDistanceEntry.getDouble(0));
        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        System.out.println("lior is white" + Point.distance(Robot.drivetrain.currentLocation, targetWP));
        PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
        pursue.start();
    }

    /**
     * @return if the robot arrived to the target
     */
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (targetDistanceEntry.getDouble(0) < 0.3);
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
     * @return path with middle waypoint to the target
     */
    private Path generateFromVision(double angle, double distance) {
        double targetDistance = distance;
        Waypoint targetWP = target(angle, targetDistance);
        Waypoint middleWP = getMiddleWP(targetWP);
        Path path1 = new Path(new Waypoint[]{new Waypoint(0, 0), middleWP, targetWP});
        path1.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        System.out.println(path1);
        return path1;
    }

    /**
     * @param target Way Point
     * @return the middle Way point
     */
    private Waypoint getMiddleWP(Waypoint target) {
        return new Waypoint(0, target.getY() - target.getY() / 2);
    }

    /**
     *
     * @param angle from target
     * @param distance from target
     * @return the target Way point
     */
    private Waypoint target(double angle, double distance) {
        return new Waypoint(Math.sin(Math.toRadians(angle)) * distance, Math.cos(Math.toRadians(angle)) * distance);
    }
}
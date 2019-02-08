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

    // Called just before this Command runs the first time
    protected void initialize() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("vision");
        targetAngleEntry = table.getEntry("angle");
        targetDistanceEntry = table.getEntry("distance");

        Path path1 = generateFromVision(targetAngleEntry.getDouble(0), targetDistanceEntry.getDouble(0));
        path1.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        SmartDashboard.putNumber("target distance", targetDistanceEntry.getDouble(0) / 100);
        SmartDashboard.putNumber("target angle ", targetAngleEntry.getDouble(0));
        PurePursue pursue = new PurePursue(path1, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
        pursue.start();
        System.out.println(path1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (Point.distance(Robot.drivetrain.currentLocation,
                middleWP(target(targetAngleEntry.getDouble(0), targetDistanceEntry.getDouble(0)))) <= 0 ||
                Point.distance(Robot.drivetrain.currentLocation, target(targetAngleEntry.getDouble(0), targetDistanceEntry.getDouble(0)))
                        >= Constants.MIN_DISTANCE) {
            Path path = generateFromVision(targetAngleEntry.getDouble(0), targetDistanceEntry.getDouble(0));
            path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
            PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
            pursue.start();
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    private Path generateFromVision(double angle, double distance) {
        double targetDistance = distance / 100;
        double targetAngle = angle;
        Waypoint target = target(targetAngle, targetDistance);
        Waypoint middleWP = middleWP(target);
        Path path1 = new Path(new Waypoint[]{new Waypoint(0, 0), middleWP, target});
        System.out.println(path1);
        return path1;
    }

    private Waypoint middleWP(Waypoint target) {
        Waypoint middleWP = new Waypoint(0, target.getY() - target.getY() / 2);
        return middleWP;
    }

    private Waypoint target(double angle, double distance) {
        double targetDistance = distance / 100;
        double targetAngle = angle;
        Waypoint target = new Waypoint(Math.sin(Math.toRadians(targetAngle)) * targetDistance + 0.15, Math.cos(Math.toRadians(targetAngle)) * targetDistance);
        return target;
    }
}
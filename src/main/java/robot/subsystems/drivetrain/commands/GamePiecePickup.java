package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.PurePursue;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

/**
 *
 */
public class GamePiecePickup extends Command {

    public GamePiecePickup() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    private NetworkTableEntry targetAngleEntry;
    private NetworkTableEntry targetDistanceEntry;


    /**
     * take distance and angle from the network tables than generate th pure pursue by that parameters
     */
    // Called just before this Command runs the first time
    protected void initialize() {
        targetAngleEntry = Robot.visionTable.getEntry("cargo_angle");
        targetDistanceEntry = Robot.visionTable.getEntry("cargo_distance");

        double targetDistance = targetDistanceEntry.getDouble(0)/100;
        double targetAngle = targetAngleEntry.getDouble(0);
        Waypoint target = new Waypoint(Math.sin(Math.toRadians(targetAngle)) * targetDistance +0.15, Math.cos(Math.toRadians(targetAngle)) * targetDistance   );
        Waypoint middleWP = new Waypoint(0, target.getY()-target.getY()/2);
        Path path1 = new Path(new Waypoint[]{new Waypoint(0,0), middleWP ,  target});
        System.out.println(path1);
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
        if (targetDistanceEntry.getDouble(0) > 0.3) {
            Path path = generateFromVision(targetAngleEntry.getDouble(0), targetDistanceEntry.getDouble(0));
            path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
            System.out.println("lior is white" + Point.distance(Robot.drivetrain.currentLocation, targetWP));
            PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
            pursue.start();
        }
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
        System.out.println("paulo shahor");
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
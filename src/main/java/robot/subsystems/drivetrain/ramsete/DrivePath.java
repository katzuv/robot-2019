package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

import static robot.Robot.drivetrain;

/**
 *
 */
public class DrivePath extends Command {

    private Path path;
    private int index = 0;
    private RamseteTracker tracker;
    private double lastLeftDistance = 0;
    private double lastRightDistance = 0;
    private double x = 0;
    private double y = 0;

    public DrivePath(Path path) {
        this.path = path;
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        drivetrain.resetEncoders();
        drivetrain.resetLocation();
        tracker = new RamseteTracker(2, 0.7);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Waypoint nextPoint;
        nextPoint = new Waypoint(0, 0, 0, 0, 0);

        double[] speeds = tracker.calculateState(RamseteTracker.waypoints().get(0), getCurrentPoint(), nextPoint);
//        System.out.println(Arrays.toString(speeds));
        Robot.drivetrain.setLeftVelocity(speeds[0]);
        Robot.drivetrain.setRightVelocity(speeds[1]);
//        index++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
//        return index >= RamseteTracker.waypoints().size();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    private Waypoint getCurrentPoint() {
        double distance = ((drivetrain.getLeftDistance() - lastLeftDistance) + (drivetrain.getRightDistance() - lastRightDistance)) / 2;

        x += distance * Math.cos(drivetrain.getAngle());
        y += distance * Math.sin(drivetrain.getAngle());
        System.out.println(x + ", " + y + "|" + drivetrain.getAngle());
        lastLeftDistance = drivetrain.getLeftDistance();
        lastRightDistance = drivetrain.getRightDistance();
        return new Waypoint(x, y);
    }
}
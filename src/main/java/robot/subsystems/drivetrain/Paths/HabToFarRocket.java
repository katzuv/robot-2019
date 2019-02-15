package robot.subsystems.drivetrain.Paths;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.PurePursue;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

/**
 *
 */
public class HabToFarRocket extends InstantCommand {

    public HabToFarRocket() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Path path = new Path(new Waypoint[]{new Waypoint(0.35, 5), (new Waypoint(0.75, 5.5))});
        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
        pursue.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
}
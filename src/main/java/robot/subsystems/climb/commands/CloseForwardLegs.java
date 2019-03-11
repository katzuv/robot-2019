package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.climb.Constants;

import static robot.Robot.climb;

/**
 *
 */
public class CloseForwardLegs extends Command {

    public CloseForwardLegs() {
        requires(climb);

        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        climb.setFrontLegHeights(0,0);
        if (climb.getLegBRHeight() > Constants.DRIVE_CLIMB_HEIGHT_THRESH || climb.getLegBLHeight() > Constants.DRIVE_CLIMB_HEIGHT_THRESH) {
            climb.setBackLegHeights(climb.getLegBLHeight() - 0.02, 0);
        }

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return climb.getLegFLHeight() < Constants.CLIMB_TOLERANCE &&
                climb.getLegFRHeight() < Constants.CLIMB_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
        climb.setLegDriveSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
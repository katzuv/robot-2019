package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.climb.Climb;
import robot.subsystems.climb.Constants;

import static robot.Robot.climb;

/**
 *
 */
public class CalibrateLegs extends Command {

    public CalibrateLegs() {
        requires(climb);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        climb.setLegBLSpeed(-Constants.CALIBRATE_SPEED);
        climb.setLegBRSpeed(-Constants.CALIBRATE_SPEED);
        climb.setLegFLSpeed(-Constants.CALIBRATE_SPEED);
        climb.setLegFRSpeed(-Constants.CALIBRATE_SPEED);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return climb.areAllLegsUp();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
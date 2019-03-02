package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.climb.Constants;
import robot.subsystems.climb.TiltUtils;

import static robot.Robot.climb;

/**
 *
 */
public class CloseLoopCalibrateLegs extends Command {

    public CloseLoopCalibrateLegs() {
        requires(climb);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        climb.resetEncoders();
        climb.setLegFLHeight(-Constants.LEVEL_THREE_LEG_LENGTH,0);
        climb.setLegFRHeight(-Constants.LEVEL_THREE_LEG_LENGTH,0);
        climb.setLegBLHeight(-Constants.LEVEL_THREE_LEG_LENGTH,0);
        climb.setLegBRHeight(-Constants.LEVEL_THREE_LEG_LENGTH,0);
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
        climb.resetEncoders();

        climb.setLegFLHeight(0,0);
        climb.setLegFRHeight(0,0);
        climb.setLegBLHeight(0,0);
        climb.setLegBRHeight(0,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
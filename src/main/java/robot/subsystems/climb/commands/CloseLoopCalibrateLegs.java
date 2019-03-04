package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.climb.Constants;

import static robot.Robot.climb;

/**
 * Calibrates all the legs to their zero position. this class raises the legs in a closed loop.
 * After all legs are raised, the encoder values are reset.
 *
 * @author paulo
 */
public class CloseLoopCalibrateLegs extends Command {

    public CloseLoopCalibrateLegs() {
        requires(climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
            climb.resetEncoders();
            climb.setLegFLHeight(-1.2*Constants.LEVEL_THREE_LEG_LENGTH, 0);
            climb.setLegFRHeight(-1.2*Constants.LEVEL_THREE_LEG_LENGTH, 0);
            climb.setLegBLHeight(-1.2*Constants.LEVEL_THREE_LEG_LENGTH, 0);
            climb.setLegBRHeight(-1.2*Constants.LEVEL_THREE_LEG_LENGTH, 0);
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

        climb.setLegFLHeight(0, 0);
        climb.setLegFRHeight(0, 0);
        climb.setLegBLHeight(0, 0);
        climb.setLegBRHeight(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.climb.Constants;

import static robot.Robot.climb;

/**
 *
 */
public class BalanceLegs extends Command {

    private final double targetHeight;
    public BalanceLegs(double targetHeight) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.targetHeight = targetHeight;
        requires(climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        climb.setLegDriveHeight(0,0);
        climb.setLegBRHeight(targetHeight, 0);
        climb.setLegBLHeight(targetHeight,0 );
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(climb.getLegFLHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegFRHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegBLHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegBRHeight() - targetHeight) < Constants.CLIMB_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
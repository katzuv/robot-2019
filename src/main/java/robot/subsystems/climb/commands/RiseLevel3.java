package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.climb.Constants;

import static robot.Robot.climb;

/**
 *
 */
public class RiseLevel3 extends Command {
    private double targetHeight = Constants.LEVEL_THREE_LEG_LENGTH;
    //gamers, lose yourself and rise up
    public RiseLevel3() {
        requires(climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        climb.setLegFLHeight(targetHeight); //TODO:think of using arbitrary feedforward
        climb.setLegFRHeight(targetHeight);
        climb.setLegBLHeight(targetHeight);
        climb.setLegBRHeight(targetHeight);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() { //TODO: either try out an average, a timeout or an angle consideration aswell
        return climb.getLegFLHeight()>targetHeight-Constants.CLIMB_TOLERANCE &&
                climb.getLegFRHeight()>targetHeight-Constants.CLIMB_TOLERANCE &&
                climb.getLegBLHeight()>targetHeight-Constants.CLIMB_TOLERANCE &&
                climb.getLegBRHeight()>targetHeight-Constants.CLIMB_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
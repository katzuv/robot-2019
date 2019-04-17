package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.climb.Constants;

import static robot.Robot.climb;

/**
 * This class is primarily used to nudge the back legs before climbing.
 *
 * @author paulo
 */
public class MoveBackLegs extends Command {
    private final double targetHeight;

    public MoveBackLegs(double targetHeight) {
        this.targetHeight = targetHeight;
        requires(Robot.climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        climb.setLegBLHeight(targetHeight, 0);
        climb.setLegBRHeight(targetHeight, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(climb.getLegBLHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
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
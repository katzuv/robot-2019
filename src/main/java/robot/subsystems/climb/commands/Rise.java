package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;

import static robot.Robot.climb;

/**
 *
 */
public class Rise extends Command {
    //gamers, lose yourself and rise up
    public Rise() {
        requires(climb);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        climb.raiseForwardLegs();
        climb.raiseBackLegs();
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
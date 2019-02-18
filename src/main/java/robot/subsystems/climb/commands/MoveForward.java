package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveForward extends Command {

    /**
     * Drive the climb wheel forward at a specified speed, until an end statement is met.
     *
     * @param velocity velocity of the wheel in meters per seconds
     * @param timeout  time in seconds until the command stops
     */
    public MoveForward(double velocity, double timeout) {

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
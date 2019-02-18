package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class MoveForward extends Command {
    private double velocity;
    private double timeout;
    private Timer timer = new Timer();
    /**
     * Drive the climb wheel forward at a specified speed, until an end statement is met.
     *
     * @param velocity velocity of the wheel in meters per seconds
     * @param timeout  time in seconds until the command stops
     */
    public MoveForward(double velocity, double timeout) {
        this.velocity = velocity;
        this.timeout = timeout;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.climb.setWheelVelocity(velocity);
        timer.reset();
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timer.get() >= timeout;
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.climb.setWheelVelocity(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
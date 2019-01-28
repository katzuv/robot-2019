package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.elevator.Elevator;

/**
 *  Move the elevator to a certain height
 */
public class LiftCommand extends Command {
    private double delta = 0.1;
    private Elevator elevator = Robot.elevator;
    private double height;

    /**
     * Make the elevator move to a specific height.
     *
     * @param height height in meters of the elevator
     */
    public LiftCommand(double height) {
        requires(elevator);
        this.height = height;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        elevator.setHeight(height);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        elevator.update();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(elevator.getHeight() - height) < delta;
    }

    // Called once after isFinished returns true
    protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        this.end();
    }
}
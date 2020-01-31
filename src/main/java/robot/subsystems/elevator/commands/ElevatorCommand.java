package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.subsystems.elevator.Constants;

import static robot.Robot.elevator; //elevator subsystem


/**
 * Move the elevator to a certain height
 */
public class ElevatorCommand extends CommandBase {
    private double tolerance = Constants.ELEVATOR_TOLERANCE;
    private double height;

    /**
     * Make the elevator move to a specific height.
     *
     * @param height height in meters of the elevator
     */
    public ElevatorCommand(double height) {
        addRequirements(elevator);
        this.height = height;
    }

    /**
     * Make the elevator move to one of the predefined heights.
     *
     * @param state an enum of heights, defined in the elevator constants class.
     */
    public ElevatorCommand(Constants.ELEVATOR_HEIGHTS state) {
        this(state.getLevelHeight());
    }

    // Called just before this Command runs the first time
    public void initialize() {
        elevator.setHeight(height);
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        elevator.update();
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return Math.abs(elevator.getHeight() - height) < tolerance;
    }

    // Called once after isFinished returns true
    protected void end() {
        elevator.update();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        this.end();
    }
}

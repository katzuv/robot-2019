package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.Elevator;

/**
 * Move the elevator to a certain height
 */
public class ElevatorCommand extends CommandBase {
    private Elevator elevator;
    private double tolerance = Constants.ELEVATOR_TOLERANCE;
    private double height;

    /**
     * Make the elevator move to a specific height.
     *
     * @param height height in meters of the elevator
     */
    public ElevatorCommand(Elevator elevator, double height) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.height = height;
    }

    /**
     * Make the elevator move to one of the predefined heights.
     *
     * @param state an enum of heights, defined in the elevator constants class.
     */
    public ElevatorCommand(Elevator elevator, Constants.ELEVATOR_HEIGHTS state) {
        this(elevator, state.getLevelHeight());
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        elevator.setHeight(height);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        elevator.update();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getHeight() - height) < tolerance;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        elevator.update();
    }
}

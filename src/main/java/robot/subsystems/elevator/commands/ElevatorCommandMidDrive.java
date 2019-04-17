package robot.subsystems.elevator.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.elevator.Constants;

import static robot.Robot.elevator;
import static robot.Robot.hatchIntake;

/**
 * Move the elevator to a certain height
 */
public class ElevatorCommandMidDrive extends Command {
    private double tolerance = Constants.ELEVATOR_TOLERANCE;
    private double height;
    NetworkTableEntry targetDistanceEntry;
    private boolean lifted = false;

    /**
     * Make the elevator move to a specific height.
     *
     * @param height height in meters of the elevator
     */
    public ElevatorCommandMidDrive(double height) {
        requires(elevator);
        this.height = height;
    }

    /**
     * Make the elevator move to one of the predefined heights.
     *
     * @param state an enum of heights, defined in the elevator constants class.
     */
    public ElevatorCommandMidDrive(Constants.ELEVATOR_STATES state) {
        this(state.getLevelHeight());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("vision");
        targetDistanceEntry = table.getEntry("distance");
        if (targetDistanceEntry.getDouble(0) <= 1 && !lifted && !hatchIntake.isGripperPlateExtended() && !hatchIntake.isGripperOpen()){
            elevator.setHeight(height);
            lifted = true;
        }

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        elevator.update();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(elevator.getHeight() - height) < tolerance;
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

package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import static robot.Robot.hatchIntake;

/*
this command controls the flower on the robot
 */
public class Flower extends InstantCommand {
    
    private gripperState current;//enum variable that indicates the current mode of the gripper

    /**
     * empty constructor, sets the wanted state to toggle meaning whenever the command is called it will toggle the current state
     * instead of going to the wanted state
     */
    public Flower() {
        requires(hatchIntake);
        current = gripperState.TOGGLE_FLOWER;

    }

    /**
     * constructor that sets the wanted state
     *
     * @param open if true changes the wanted state to open and otherwise sets the wanted state to cloes
     */
    public Flower(boolean open) {
        requires(hatchIntake);
        if (open)
            current = gripperState.GRIPPER_GRAB;
        else
            current = gripperState.GRIPPER_RELEASE;
    }

    @Override
    public void initialize() {
        switch (current) {
            case TOGGLE_FLOWER: // Change to the second state
                hatchIntake.setGripper(!hatchIntake.isGripperOpen());
                break;
            case GRIPPER_GRAB: // Open the gripper if closed and not do anything otherwise
                hatchIntake.setGripper(true);
                break;
            case GRIPPER_RELEASE:// Close the gripper if opened and not do anything otherwise
                hatchIntake.setGripper(false);
                break;
        }
    }

    @Override
    public void execute() {

    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }

    /**
     * enum to indicate the state of the gripper
     */
    public enum gripperState {
        TOGGLE_FLOWER,
        GRIPPER_GRAB,
        GRIPPER_RELEASE
    }
}
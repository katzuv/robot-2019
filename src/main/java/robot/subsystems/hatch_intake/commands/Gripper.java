package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.hatch_intake.HatchIntake;

public class Gripper extends Command {
    private boolean open;

    private boolean open;//indicates whether the flower is open or not
    private HatchIntake hatchIntake = new HatchIntake();
    private opMode current;//enum variable that indicates the current mode of the gripper


    /**
     * empty constructor, sets the wanted state to toggle meaning whenever the command is called it will toggle the current state
     * instead of going to the wanted state
     */
    public Gripper() {
        requires(Robot.hatchIntake);
        current = opMode.TOGGLE;

    }

    /**
     * constructor that sets the wanted state
     *
     * @param open if true changes the wanted state to open and otherwise sets the wanted state to cloes
     */
    public Gripper(boolean open) {
        requires(Robot.hatchIntake);
        if (open)
            current = opMode.OPEN;
        else
            current = opMode.CLOSE;
    }

    @Override
    public void initialize() {
        switch (current) {
            case TOGGLE: // Change to the second state
                hatchIntake.setGripper();
                break;
            case OPEN: // Open the gripper if closed and not do anything otherwise
                if (!hatchIntake.isGripperOpen())
                    hatchIntake.setGripper();
                break;
            case CLOSE:// Close the gripper if opened and not do anything otherwise
                if (hatchIntake.isGripperOpen())
                    hatchIntake.setGripper();
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
    public enum opMode {
        TOGGLE,
        OPEN,
        CLOSE
    }
}

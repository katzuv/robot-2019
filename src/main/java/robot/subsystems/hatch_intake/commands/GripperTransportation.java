package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

import static robot.Robot.hatchIntake;

public class GripperTransportation extends InstantCommand {
    private boolean extend;
    private gripperPlateState current;//enum variable that indicates the current mode of the gripperPlate

    public GripperTransportation(boolean extend) {
        requires(Robot.hatchIntake);
        if (extend)
            current = gripperPlateState.GRIPPER_PLATE_EXTEND;
        else
            current = gripperPlateState.GRIPPER_PLATE_PULL;
    }
    public GripperTransportation() {
        requires(hatchIntake);
        current = gripperPlateState.TOGGLE_GRIPPER_PLATE;

    }

    @Override
    public void initialize() {
        switch (current) {
            case TOGGLE_GRIPPER_PLATE: // Change to the second state
                hatchIntake.setGripperPlate();
                break;
            case GRIPPER_PLATE_EXTEND: // extend the gripperPlate if closed and not do anything otherwise
                if (!hatchIntake.isGripperPlateExtended())
                    hatchIntake.setGripperPlate();
                break;
            case GRIPPER_PLATE_PULL:// pull the gripperplate back if extended and not do anything otherwise
                if (hatchIntake.isGripperPlateExtended())
                    hatchIntake.setGripperPlate();
                break;
        }

    }

    @Override
    public void execute() {

    }


    @Override
    protected void end() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }

    public enum gripperPlateState {
        TOGGLE_GRIPPER_PLATE,
        GRIPPER_PLATE_EXTEND,
        GRIPPER_PLATE_PULL
    }
}

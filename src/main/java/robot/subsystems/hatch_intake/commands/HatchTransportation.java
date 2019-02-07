package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class HatchTransportation extends InstantCommand { //TODO: Refactor transportation to a better name
    private hatchPickupState current;

    public HatchTransportation(boolean down) {
        requires(Robot.hatchIntake);
        if (down)
            current = hatchPickupState.HATCH_DOWN;
        else
            current = hatchPickupState.HATCH_UP;
    }

    public HatchTransportation() {
        requires(Robot.hatchIntake);
        current = hatchPickupState.TOGGLE_HATCH_PICKUP_STATE;
    }

    @Override
    public void initialize() {
        switch (current) {
            case TOGGLE_HATCH_PICKUP_STATE:
                Robot.hatchIntake.setGroundIntake(!Robot.hatchIntake.isHatchDown());
                break;
            case HATCH_DOWN:
                Robot.hatchIntake.setGripperPlate(true);
                break;
            case HATCH_UP:
                Robot.hatchIntake.setGripperPlate(false);
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

    public enum hatchPickupState {
        TOGGLE_HATCH_PICKUP_STATE,
        HATCH_UP,
        HATCH_DOWN
    }
}

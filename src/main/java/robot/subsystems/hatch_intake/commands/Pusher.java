package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class Pusher extends InstantCommand { //TODO: Refactor transportation to a better name
    private extensionPlateState current;//enum variable that indicates the current mode of the extensionPlate

    public Pusher(boolean extend) {
        requires(Robot.hatchIntake);
        if (extend)
            current = extensionPlateState.EXTENSION_PLATE_EXTEND;
        else
            current = extensionPlateState.EXTENSION_PLATE_PULL;
    }

    public Pusher() {
        requires(Robot.hatchIntake);
        current = extensionPlateState.TOGGLE_EXTENSION_PLATE;
    }

    @Override
    public void initialize() {
        switch (current) {
            case TOGGLE_EXTENSION_PLATE: // Change to the second state
                Robot.hatchIntake.setPusher(!Robot.hatchIntake.isPusherExtended());
                break;
            case EXTENSION_PLATE_EXTEND: // extend the extensionPlate if closed and not do anything otherwise
                Robot.hatchIntake.setPusher(true);
                break;
            case EXTENSION_PLATE_PULL:// pull the extensionPlate back if extended and not do anything otherwise
                Robot.hatchIntake.setPusher(false);
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

    public enum extensionPlateState {
        TOGGLE_EXTENSION_PLATE,
        EXTENSION_PLATE_EXTEND,
        EXTENSION_PLATE_PULL
    }

}

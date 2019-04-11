package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class Pusher extends InstantCommand { //TODO: Refactor transportation to a better name
    private pusherState current;//enum variable that indicates the current mode of the pusher

    /**
     * this command is used to either extend or retract the pusher
     * @param extend if true the pusher will extend and if false it will retract
     */

    public Pusher(boolean extend) {
        requires(Robot.hatchIntake);
        if (extend)
            current = pusherState.EXTENSION_PUSHER;
        else
            current = pusherState.RETRACT_PUSHER;
    }

    /**
     * empty constructor, will toggle if used
     */

    public Pusher() {
        requires(Robot.hatchIntake);
        current = pusherState.TOGGLE_PUSHER;
    }

    @Override
    public void initialize() {
        switch (current) {
            case TOGGLE_PUSHER: // Change to the second state
                Robot.hatchIntake.setPusher(!Robot.hatchIntake.isPusherExtended());
                break;
            case EXTENSION_PUSHER: // extend the pusher if closed and not do anything otherwise
                Robot.hatchIntake.setPusher(true);
                break;
            case RETRACT_PUSHER:// pull the pusher back if extended and not do anything otherwise
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

    public enum pusherState {
        TOGGLE_PUSHER,
        EXTENSION_PUSHER,
        RETRACT_PUSHER
    }

}

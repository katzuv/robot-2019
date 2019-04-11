package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class Fangs extends InstantCommand { //TODO: Refactor transportation to a better name
    private fangState current;//enum variable that indicates the current mode of the pusher

    /**
     * this command is used to either extend or retract the pusher
     * @param extend if true the pusher will extend and if false it will retract
     */

    public Fangs(boolean extend) {
        requires(Robot.hatchIntake);
        if (extend)
            current = fangState.EXTEND_FANGS;
        else
            current = fangState.RETRACT_FANGS;
    }

    /**
     * empty constructor, will toggle if used
     */

    public Fangs() {
        requires(Robot.hatchIntake);
        current = fangState.TOGGLE_FANGS;
    }

    @Override
    public void initialize() {
        switch (current) {
            case TOGGLE_FANGS: // Change to the second state
                Robot.hatchIntake.setPusher(!Robot.hatchIntake.isPusherExtended());
                break;
            case EXTEND_FANGS: // extend the pusher if closed and not do anything otherwise
                Robot.hatchIntake.setPusher(true);
                break;
            case RETRACT_FANGS:// pull the pusher back if extended and not do anything otherwise
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

    public enum fangState {
        TOGGLE_FANGS,
        EXTEND_FANGS,
        RETRACT_FANGS
    }

}

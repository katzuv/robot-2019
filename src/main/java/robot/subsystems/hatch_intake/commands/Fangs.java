package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class Fangs extends Command {
    private final double timeout;
    private Timer timer = new Timer();
    private fangState current;//enum variable that indicates the current mode of the pusher

    /**
     * this command is used to either extend or retract the pusher
     *
     * @param extend if true the pusher will extend and if false it will retract
     */
    public Fangs(boolean extend, double timeout) {
        requires(Robot.hatchIntake);
        if (extend)
            current = fangState.EXTEND_FANGS;
        else
            current = fangState.RETRACT_FANGS;
        this.timeout = timeout;
    }

    /**
     * push out fangs if used.
     */
    public Fangs() {
        this(true, 0);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        switch (current) {
            case EXTEND_FANGS: // extend the pusher if closed and not do anything otherwise
                Robot.hatchIntake.setFangs(true);
                break;
            case RETRACT_FANGS:// pull the pusher back if extended and not do anything otherwise
                Robot.hatchIntake.setFangs(false);
                break;
        }

    }

    @Override
    public void execute() {

    }

    @Override
    protected void end() {
        Robot.hatchIntake.setFangs(false);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > timeout;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

    public enum fangState {
        EXTEND_FANGS,
        RETRACT_FANGS
    }


}

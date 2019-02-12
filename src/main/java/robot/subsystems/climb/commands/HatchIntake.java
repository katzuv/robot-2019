package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class HatchIntake extends Command {
    private HATCH_INTAKE_STATES state;

    public HatchIntake(boolean down) {
        requires(Robot.climb);
        if (down)
            state = HATCH_INTAKE_STATES.MOVE_DOWN;
        else
            state = HATCH_INTAKE_STATES.MOVE_UP;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        switch(state){
            case MOVE_DOWN:
                Robot.climb.setHatchIntake(true);
                break;
            case MOVE_UP:
                Robot.climb.setHatchIntake(false);
                break;
        }
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    private enum HATCH_INTAKE_STATES {
        MOVE_DOWN,
        MOVE_UP;
    }
}
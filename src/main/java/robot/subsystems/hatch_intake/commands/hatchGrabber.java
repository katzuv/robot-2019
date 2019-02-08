package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class hatchGrabber extends Command {

    public hatchGrabber() {
        requires(Robot.hatchIntake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    protected void end() {
        Robot.hatchIntake.setGroundIntake(true);// At the end pick up the ground intake
    }

    @Override
    public boolean isFinished() {
        return Robot.hatchIntake.isHatchInside(); //End command when hatch is inside
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }

}

package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class FoldCommand extends Command {

    public FoldCommand() {
        requires(Robot.groundintake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //if hatch inside transport it to the flower
        if (Robot.groundintake.isInside()) {
            Robot.groundintake.setClose();
        } else {
            Robot.groundintake.setOpen();
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}

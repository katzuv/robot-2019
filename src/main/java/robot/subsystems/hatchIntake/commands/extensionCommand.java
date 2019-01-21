package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class extensionCommand extends Command {
    private boolean exetend;

    public extensionCommand(boolean extend) {
        requires(Robot.flowerExtension);
        this.exetend = exetend;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (exetend) {
            Robot.flowerExtension.setOpen();
        } else {
            Robot.flowerExtension.setClose();
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

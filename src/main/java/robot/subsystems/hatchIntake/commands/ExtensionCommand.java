package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class ExtensionCommand extends Command {
    private boolean exetend;

    public ExtensionCommand(boolean extend) {
        requires(Robot.groundintake);
        this.exetend = exetend;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (exetend) {
            Robot.groundintake.setExtensionOpen();
        } else {
            Robot.groundintake.setExtensionClose();
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

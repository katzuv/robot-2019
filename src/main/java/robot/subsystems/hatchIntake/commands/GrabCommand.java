package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class GrabCommand extends Command {
    private boolean open;

    public GrabCommand(boolean open) {
        requires(Robot.groundintake);
        this.open = open;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (open) {
            Robot.groundintake.setFlowerOpen();//grab hatch
        } else {
            Robot.groundintake.setFlowerClose();//release hatch
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

package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class flowerCommand extends Command {
    private boolean open;

    public flowerCommand(boolean open) {
        requires(Robot.flower);
        this.open = open;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (open) {
            Robot.flower.setOpen();
        } else {
            Robot.flower.setClose();
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

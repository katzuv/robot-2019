package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class FlowerTransportation extends Command {
    private boolean exetend;

    public FlowerTransportation(boolean extend) {
        requires(Robot.GROUNDINTAKE);
        this.exetend = extend;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (exetend) {
            Robot.GROUNDINTAKE.ExtensionOpen();
        } else {
            Robot.GROUNDINTAKE.ExtensionClose();
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

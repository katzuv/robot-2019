package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class HatchTransportation extends Command {

    public HatchTransportation() {
        requires(Robot.GROUNDINTAKE);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //if hatch inside transport it to the flower
        if (Robot.GROUNDINTAKE.isInside()) {
            Robot.GROUNDINTAKE.Close();
        } else {
            Robot.GROUNDINTAKE.Open();
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

package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class HatchTransportation extends InstantCommand {

    public HatchTransportation() {
        requires(Robot.hatchIntake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //if hatch inside transport it to the flower
        if (Robot.hatchIntake.isHatchInside()) {
            Robot.hatchIntake.closeIntake();
        } else {
            Robot.hatchIntake.openIntake();
        }
    }


    @Override
    protected void end() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}

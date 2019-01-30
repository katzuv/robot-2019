package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class HatchTransportation extends InstantCommand {

    public HatchTransportation() {
        requires(Robot.GROUNDINTAKE);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //if hatch inside transport it to the flower
        if (Robot.GROUNDINTAKE.isHatchInside()) {
            Robot.GROUNDINTAKE.closeIntake();
        } else {
            Robot.GROUNDINTAKE.openIntake();
        }
    }


    @Override
    protected void end() {
    }

    @Override
    public boolean isFinished() {
        return Robot.GROUNDINTAKE.HaveGamePiece();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}

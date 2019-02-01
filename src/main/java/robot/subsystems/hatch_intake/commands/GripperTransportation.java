package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class GripperTransportation extends InstantCommand {
    private boolean extend;

    public GripperTransportation(boolean extend) {
        requires(Robot.GROUNDINTAKE);
        this.extend = extend;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //if extend is true it extend the gripper else it bring it back
        if (extend) {
            Robot.GROUNDINTAKE.extensionOpen();
        } else {
            Robot.GROUNDINTAKE.extensionClose();
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

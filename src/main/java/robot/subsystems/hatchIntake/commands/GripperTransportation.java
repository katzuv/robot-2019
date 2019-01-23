package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;
/*
this command is used to extend and bring back the gripper
 */
public class GripperTransportation extends InstantCommand {
    private boolean extended;

    public GripperTransportation(boolean extend) {
        requires(Robot.GROUNDINTAKE);
        this.extended = extend;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //if extended it brings the gripper back and extends it otherwise
        if (extended) {
            Robot.GROUNDINTAKE.ExtensionClose();
        } else {
            Robot.GROUNDINTAKE.ExtensionOpen();
        }
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

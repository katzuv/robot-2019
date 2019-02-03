package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class GripperTransportation extends InstantCommand {
    private boolean extend;

    public GripperTransportation(boolean extend) {
        requires(Robot.hatchIntake);
        this.extend = extend;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //if extend is true it extend the gripper else it bring it back
        if (extend) {
            Robot.hatchIntake.gripperPlateOpen();
        } else {
            Robot.hatchIntake.gripperPlateClose();
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

package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/*
this command controls the flower on the robot
 */
public class Gripper extends Command {

    private boolean open;//indicates whether the flower is open or not

    public Gripper(boolean open) {
        requires(Robot.hatchIntake);
        this.open = open;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (open) {
            Robot.hatchIntake.gripperOpen();//grab hatch
        } else {
            Robot.hatchIntake.gripperClose();//release hatch
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

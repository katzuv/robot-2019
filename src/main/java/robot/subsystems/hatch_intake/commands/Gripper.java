package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;
import robot.subsystems.hatch_intake.HatchIntake;

import static robot.Robot.hatchIntake;

/*
this command controls the flower on the robot
 */
public class Gripper extends InstantCommand {




    /**
     * empty constructor, sets the wanted state to toggle meaning whenever the command is called it will toggle the current state
     * instead of going to the wanted state
     */
    public Gripper() {
        requires(Robot.hatchIntake);


    }

    @Override
    public void initialize() {
        if (hatchIntake.isGripperOpen())
            hatchIntake.closeGripper();
        else
            hatchIntake.openGripper();
    }


    @Override
    public void execute() {

    }

    @Override
    protected boolean isFinished() {
        return true;
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

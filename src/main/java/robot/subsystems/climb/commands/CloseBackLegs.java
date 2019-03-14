package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.climb.Constants;
import robot.subsystems.climb.TiltUtils;

import static robot.Robot.climb;

/**
 *
 */
public class CloseBackLegs extends Command {

    public CloseBackLegs() {
        requires(climb);

        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        climb.setLegBLHeight(0,0);
        climb.setLegBRHeight(0,0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return  climb.getLegBLHeight() < Constants.CLIMB_TOLERANCE &&
                climb.getLegBRHeight() < Constants.CLIMB_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {

        climb.setLegBLSpeed(-0.1);
        climb.setLegBRSpeed(-0.1);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
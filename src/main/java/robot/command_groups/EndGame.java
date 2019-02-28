package robot.command_groups;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitUntilCommand;
import robot.Robot;

/**
 *
 */
public class EndGame extends WaitUntilCommand {


    public EndGame(double time) {
        super(time);
        requires(Robot.cargoIntake);
        requires(Robot.hatchIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.hatchIntake.setGripper(false);
        Robot.hatchIntake.setGripperPlate(false);
        Robot.cargoIntake.setWristAngle(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
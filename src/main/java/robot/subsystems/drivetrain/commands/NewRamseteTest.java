package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.Point;
import robot.subsystems.drivetrain.ramsete.RamseteNew;

/**
 *the new Ramsete code test command
 */
public class NewRamseteTest extends Command {
    Path path = new Path();//need to add path for testing
    private RamseteNew ramseteNew = new RamseteNew(path);


    public NewRamseteTest() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    ramseteNew.getNextVelocities();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return ramseteNew.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */

public class PursuePath extends Command {
    private Point RobotCurrent = new Point(0,0);
    public PursuePath() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        RobotCurrent.setX(Math.cos(Robot.navx.getAngle())*(Robot.drivetrain.getLeftDistance()+Robot.drivetrain.getRightDistance())/2);
        RobotCurrent.setY(Math.sin(Robot.navx.getAngle())*(Robot.drivetrain.getLeftDistance()+Robot.drivetrain.getRightDistance())/2);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
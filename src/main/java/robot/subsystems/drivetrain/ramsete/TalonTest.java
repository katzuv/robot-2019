package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static robot.Robot.driveType;
import static robot.Robot.drivetrain;

/**
 *
 */
public class TalonTest extends Command {

    public TalonTest() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        drivetrain.setLeftVelocity(1);
        drivetrain.setRightVelocity(1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        SmartDashboard.putNumber("leftVelocity", drivetrain.getLeftVelocity());
        SmartDashboard.putNumber("rightVelocity", drivetrain.getRightVelocity());
        System.out.println(drivetrain.getLeftVelocity() + "|" + drivetrain.getRightVelocity());
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
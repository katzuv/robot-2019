package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class DriveToPosition extends Command {
    private double targetLeftDistance;
    private double targetRightDistance;
    private double initialLeftDistance;
    private double initialRightDistance;

    public DriveToPosition(double left, double right) {
        targetLeftDistance = left;
        targetRightDistance = right;
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        initialLeftDistance = Robot.drivetrain.getLeftPosition();
        initialRightDistance = Robot.drivetrain.getRightPosition();
        Robot.drivetrain.setPosition(targetLeftDistance + initialLeftDistance, targetRightDistance + initialRightDistance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
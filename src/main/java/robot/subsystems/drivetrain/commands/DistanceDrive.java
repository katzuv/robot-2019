package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;

import static robot.Robot.drivetrain;

/**
 *
 */
public class DistanceDrive extends Command {
    private double distance;
    public DistanceDrive(double distance) {
        requires(drivetrain);
        this.distance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        drivetrain.driveDistance(distance); //TODO: currently this doesn't use the real values
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
        drivetrain.setSpeed(0,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
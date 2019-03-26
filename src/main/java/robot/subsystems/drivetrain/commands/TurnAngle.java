package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;

import static robot.Robot.drivetrain;
import static robot.Robot.navx;

/**
 *
 */
public class TurnAngle extends Command {
    private double angle;
    public TurnAngle(double angle) {
        this.angle = navx.getAngle()+angle;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        drivetrain.setVelocity(1.5, -1.5);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return this.angle-navx.getAngle() < 10;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

/**
 * Drive a given distance on both sides of the Drivetrain.
 */
public class DistanceDrive extends Command {
    private double distance;
    private double startDistance;

    public DistanceDrive(double distance) {
        requires(drivetrain);
        this.distance = distance;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        startDistance = (drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2;
        drivetrain.driveDistance(distance); //TODO: currently this doesn't use the real values
        drivetrain.setMotorsToBrake();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs((distance + startDistance) - ((drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2)) < Constants.ENDING_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
        drivetrain.setMotorsToCoast();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
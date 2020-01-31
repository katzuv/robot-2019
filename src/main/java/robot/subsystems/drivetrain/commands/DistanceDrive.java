package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

/**
 * This command drives a specified distance using motion magic.
 */
public class DistanceDrive extends CommandBase {
    private double distance;
    private double startDistance;

    public DistanceDrive(double distance) {
        addRequirements(drivetrain);
        this.distance = distance;
    }

    public void initialize() {
        startDistance = (drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2;
        drivetrain.driveDistance(distance, distance); //TODO: currently this doesn't use the real values
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return Math.abs((distance + startDistance) - ((drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2)) < Constants.ENDING_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
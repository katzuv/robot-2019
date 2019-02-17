package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

/**
 *
 */
public class TurnByAngle extends Command {
    private double distance;
    private double initialLeftDistance;
    private double initialRightDistance;

    public TurnByAngle(double angle) {
        requires(drivetrain);
        distance = Math.toRadians(angle) * Constants.ROBOT_WIDTH / 2;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        this.initialLeftDistance = drivetrain.getLeftPosition();
        this.initialRightDistance = drivetrain.getRightPosition();
        drivetrain.setPosition(distance + initialLeftDistance,  - distance + initialRightDistance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }


    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {

        return Math.abs(drivetrain.getLeftPosition() - (initialLeftDistance + distance)) < Math.toRadians(Constants.DRIVE_TOLERANCE) * Constants.ROBOT_WIDTH / 2 &&
                Math.abs(drivetrain.getRightPosition() - (initialRightDistance - distance)) < Math.toRadians(Constants.DRIVE_TOLERANCE) * Constants.ROBOT_WIDTH / 2 ;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        cancel();
    }
}
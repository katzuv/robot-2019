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
        distance = Math.PI * Constants.ROBOT_WIDTH * (angle / 360);
        this.initialLeftDistance = drivetrain.getLeftPosition();
        this.initialRightDistance = drivetrain.getRightPosition();
        requires(drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        drivetrain.setPosition(distance, -distance);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }


    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs((drivetrain.getLeftPosition() - initialLeftDistance) - distance) < Constants.ROTATION_TOLERANCE &&
                Math.abs((drivetrain.getRightPosition() - initialRightDistance) - distance) < Constants.ROTATION_TOLERANCE ;
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
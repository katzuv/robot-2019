package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

/**
 *
 */
public class ProportionalDrive extends Command {

    private NetworkTableEntry targetAngleEntry;
    private NetworkTableEntry targetDistanceEntry;

    public ProportionalDrive() {
        requires(drivetrain);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        targetAngleEntry = Robot.visionTable.getEntry("tape_angle");
        targetDistanceEntry = Robot.visionTable.getEntry("tape_distance");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double targetAngle = targetAngleEntry.getDouble(0);
        double targetDistance = targetDistanceEntry.getDouble(0);
        double headingError = -targetAngle;
        double distanceError = -targetDistance;
        double steeringAdjust = 0;
        if (targetAngle > 1) {
            steeringAdjust = Constants.KpAim * headingError - Constants.MIN_AIM_COMMAND;
        } else if (targetAngle < 1) {
            steeringAdjust = Constants.KpAim * headingError + Constants.MIN_AIM_COMMAND;
        }
        double distanceAdjust = Constants.KpDistance * distanceError;
        double leftSpeed = -1 * (steeringAdjust + distanceAdjust);
        double rightSpeed = steeringAdjust + distanceAdjust;
        System.out.println(leftSpeed + " | " + rightSpeed);
        drivetrain.setSpeed(leftSpeed, rightSpeed);
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
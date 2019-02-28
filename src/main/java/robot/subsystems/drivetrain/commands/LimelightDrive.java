package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;

import static robot.Robot.drivetrain;
import static robot.Robot.networkTableInstance;

/**
 *
 */
public class LimelightDrive extends Command {

    private NetworkTableEntry distanceEntry;
    private NetworkTableEntry angleEntry;
    public LimelightDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(drivetrain);

        NetworkTable visionTable = networkTableInstance.getTable("vision");
        distanceEntry = visionTable.getEntry("tape_distance");
        angleEntry = visionTable.getEntry("tape_angle");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double angle = angleEntry.getDouble(0);
        } else if (angle > 10) { // Need to rotate left
            drivetrain.setSpeed(0.3, -0.3);
        } else if (angle < -10) { // Need to rotate right
            drivetrain.setSpeed(-0.3, 0.3);
        } else {
            // Robot is roughly aligned, so drive forward slowly
            drivetrain.setSpeed(0.3, 0.3);
        }

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
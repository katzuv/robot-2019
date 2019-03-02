package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

import static robot.Robot.drivetrain;

public class SimplePDrive extends Command {

    private NetworkTableEntry distanceEntry;
    private NetworkTableEntry angleEntry;
    private double KpAim = 0.1;
    private double KpDistance = 0.1;
    private double min_aim_command = 0.05;

    private double left_command = 0;
    private double right_command = 0;

    public SimplePDrive() {
        requires(drivetrain);

        distanceEntry = Robot.visionTable.getEntry("tape_distance");
        angleEntry = Robot.visionTable.getEntry("tape_angle");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        left_command = 0;
        right_command = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double heading_error = angleEntry.getDouble(0);
        double distance_error = distanceEntry.getDouble(0);
        double steering_adjust = 0;

        if (heading_error > 1) {
            steering_adjust = KpAim * heading_error - min_aim_command;
        } else if (heading_error < 1) {
            steering_adjust = KpAim * heading_error + min_aim_command;
        }

        double distance_adjust = KpDistance * distance_error;

        left_command += steering_adjust + distance_adjust;
        right_command -= steering_adjust + distance_adjust;

        drivetrain.setSpeed(left_command, right_command);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return distanceEntry.getDouble(0) < 0.3;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0); // Stop Genesis when it reached the target
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}

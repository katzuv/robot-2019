package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

import static robot.Robot.drivetrain;

public class SimpleVisionDrive extends Command {

    private NetworkTableEntry distanceEntry;
    private NetworkTableEntry angleEntry;
    private Timer stopTimer;
    private boolean hasTimerStarted = false;

    public SimpleVisionDrive() {
        requires(drivetrain);

        distanceEntry = Robot.visionTable.getEntry("tape_distance");
        angleEntry = Robot.visionTable.getEntry("tape_angle");
        stopTimer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        stopTimer.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double angle = angleEntry.getDouble(0);
        if (distanceEntry.getDouble(0) < 0.25) {
            if (!hasTimerStarted) {
                // If the distance is less than 25cm, the camera doesn't see the target anymore, so start to finish the command
                stopTimer.start();
                hasTimerStarted = true;
            }
            drivetrain.setSpeed(0.3, 0.3);
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
        return stopTimer.get() > 0.25; // It takes 0.25 seconds for Genesis to finally reach the target and align to it
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0); // Stop Genesis when it reached the target
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}

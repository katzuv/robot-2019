package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

/**
 *
 */
public class AngleDrive extends Command {
    private NetworkTableEntry targetAngleEntry;
    private NetworkTableEntry targetDistanceEntry;
    private boolean stop = false;
    private boolean finish = false;

    public AngleDrive() {
        requires(drivetrain);
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        finish = false;
        stop = false;
        targetAngleEntry = Robot.visionTable.getEntry("tape_angle");
        targetDistanceEntry = Robot.visionTable.getEntry("tape_distance");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        System.out.println(Math.abs(Robot.m_oi.rightSideAxis()));
        if (Math.abs(Robot.m_oi.rightSideAxis()) > 0.2) {
            finish = true;
        }
        double angle = targetAngleEntry.getDouble(0);
        double distance = targetDistanceEntry.getDouble(0);
        double speed = -1 * Robot.m_oi.leftDriveStick();
        double turn = Constants.angleKp * Math.toRadians(angle);
        if (distance != 0 && distance < 0.6) {
            stop = true;
        }
        if (stop) {
            turn = 0;
        }
        drivetrain.setSpeed(speed + turn, speed - turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finish;
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
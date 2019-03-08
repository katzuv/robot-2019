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
    private double lastAngle = 0;
    private boolean stop = false;

    public AngleDrive() {
        requires(drivetrain);
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        stop = false;
        lastAngle = 0;
        targetAngleEntry = Robot.visionTable.getEntry("tape_angle");
        targetDistanceEntry = Robot.visionTable.getEntry("tape_distance");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double angle = targetAngleEntry.getDouble(0);
        double distance = targetDistanceEntry.getDouble(0);
        double speed = -1 * Robot.m_oi.leftDriveStick();
//        double d = Constants.angleKd * (angle - lastAngle) / 0.02;
        double turn = Constants.angleKp * Math.toRadians(angle);
        if (distance != 0 && distance < 0.6) {
            stop = true;
        }
        if (stop) {
            turn = 0;
        }
        System.out.println(turn);
        drivetrain.setSpeed(speed + turn, speed - turn);
        lastAngle = angle;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
//        double distance = targetDistanceEntry.getDouble(0);
//        return distance != 0 && distance < 1.1;
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
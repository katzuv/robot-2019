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

    public AngleDrive() {
        requires(drivetrain);
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        targetAngleEntry = Robot.visionTable.getEntry("tape_angle");
        targetDistanceEntry = Robot.visionTable.getEntry("tape_distance");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double angle = targetAngleEntry.getDouble(0);
        double distance = targetDistanceEntry.getDouble(0);
        double speed = -1 * Robot.m_oi.leftDriveStick();
        if (angle == 0) {
            drivetrain.setSpeed(0, 0);
        } else {
            double turn = Constants.angleKp * Math.toRadians(angle);
            if (distance < 1) {
                turn = 0;
            }
            System.out.println(turn);
            drivetrain.setSpeed(speed + turn, speed - turn);
        }
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
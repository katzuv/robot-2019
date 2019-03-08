package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

public class GyroAngleDrive extends Command {
    private NetworkTableEntry targetAngleEntry;
    private double setpoint;

    public GyroAngleDrive() {
        requires(drivetrain);
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        targetAngleEntry = Robot.visionTable.getEntry("tape_angle");
        setpoint = drivetrain.getAngle() - targetAngleEntry.getDouble(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double error = setpoint - drivetrain.getAngle();
        double speed = -1 * Robot.m_oi.leftDriveStick();
        double turn = Constants.angleKp * Math.toRadians(error);
        System.out.println(turn);
        drivetrain.setSpeed(speed + turn, speed - turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
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
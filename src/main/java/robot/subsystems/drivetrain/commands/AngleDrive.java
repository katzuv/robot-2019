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

    public AngleDrive() {
        requires(drivetrain);
    }

    protected void initialize() {
        stop = false;
        targetAngleEntry = Robot.visionTable.getEntry("tape_angle");
        targetDistanceEntry = Robot.visionTable.getEntry("tape_distance");
    }

    protected void execute() {
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

    protected boolean isFinished() {
        return Math.abs(Robot.m_oi.rightSideAxis()) > 0.2;
    }

    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    protected void interrupted() {
        end();
    }
}
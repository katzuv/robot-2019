package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

/**
 * Autonomously drive to a vision target.
 */
public class VisionDrive extends Command {
    private MiniPID turnPid = new MiniPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");
    private double visionAngle;
    private double visionDistance;

    public VisionDrive() {
        turnPid.setOutputLimits(-1, 1);

        requires(drivetrain);
    }

    protected void initialize() {
        updateConstants();
    }

    protected void execute() {
        visionAngle = angleEntry.getDouble(0);
        visionDistance = distanceEntry.getDouble(0);

        double turn = turnPid.getOutput(visionAngle, 0);

        if (visionAngle > 1)
            turn += Constants.MIN_AIM;
        else if (visionAngle < -1)
            turn -= Constants.MIN_AIM;

        drivetrain.setLeftDistanceAndFeedForward(visionDistance - Constants.visionOffset, turn);
        drivetrain.setRightDistanceAndFeedForward(visionDistance - Constants.visionOffset, -turn);
    }

    protected boolean isFinished() {
        return !seenEntry.getBoolean(false);
    }

    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    protected void interrupted() {
        end();
    }

    public void updateConstants() {
        turnPid.setPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);
    }
}
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
    /*
            For now have all the constants here for testing, when done move to constants file
         */
    private double TARGET_VISION_DISTANCE = 1.2; //distance from the target to stop

    private MiniPID turnPid = new MiniPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");

    public VisionDrive() {
        turnPid.setOutputLimits(-0.5, 0.5);
        requires(drivetrain);
    }

    protected void initialize() {
        updateConstants();
    }

    protected void execute() {
        double visionAngle = angleEntry.getDouble(0);
        double visionDistance = distanceEntry.getDouble(0);

        double speed = Constants.VISION_SPEED;
        double turn = turnPid.getOutput(visionAngle, 0);

        if (visionAngle > 1) {
            turn -= Constants.MIN_AIM;
        } else if (visionAngle < -1) {
            turn += Constants.MIN_AIM;
        }

        if (seenEntry.getBoolean(false)) {
            drivetrain.setSpeed(speed - turn, speed + turn);
        }
    }

    protected boolean isFinished() {
        return !seenEntry.getBoolean(false) || distanceEntry.getDouble(0) < TARGET_VISION_DISTANCE;
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
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
    private double TARGET_VISION_DISTANCE = 1.05; //distance from the target to stop
    private static final double EXIT_TOLERANCE = 0.23;
    private MiniPID turnPid = new MiniPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);
    private MiniPID forwardPid = new MiniPID(Constants.PIDVisionForward[0], Constants.PIDVisionForward[1], Constants.PIDVisionForward[2]);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");

    public VisionDrive() {
        turnPid.setOutputLimits(-0.5, 0.5);
        forwardPid.setOutputLimits(-0.5, 0.5);
        forwardPid.setDirection(true);
        requires(drivetrain);
    }

    protected void initialize() {
        updateConstants();
    }

    protected void execute() {
        double visionAngle = angleEntry.getDouble(0);
        double visionDistance = distanceEntry.getDouble(0);

        double speed = forwardPid.getOutput(visionDistance, TARGET_VISION_DISTANCE);
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
        return !seenEntry.getBoolean(false) || distanceEntry.getDouble(0) < TARGET_VISION_DISTANCE + EXIT_TOLERANCE;
    }

    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    protected void interrupted() {
        end();
    }

    public void updateConstants() {
        turnPid.setPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);
        forwardPid.setPID(Constants.PIDVisionForward[0], Constants.PIDVisionForward[1], Constants.PIDVisionForward[2]);
    }
}
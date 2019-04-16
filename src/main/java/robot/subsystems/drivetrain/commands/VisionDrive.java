package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
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
    private Timer timeout = new Timer(); //this timer is meant to prevent jumps where the vision target gets lost

    private double TIMER_DELAY = 0.1;

    public VisionDrive() {
        turnPid.setOutputLimits(-0.5, 0.5);
        requires(drivetrain);
    }

    public static double velocityByDistance(double targetSpeed, double acceleration, double startPos, double targetPos) {
        return Math.sqrt(targetSpeed * targetSpeed + 2 * Math.abs(acceleration) * Math.abs(targetPos - startPos));
    }

    protected void initialize() {
        updateConstants();
    }

    protected void execute() {
        double visionAngle = angleEntry.getDouble(0);
        double visionDistance = distanceEntry.getDouble(0);

        double speed = Math.min(Constants.VISION_SPEED, velocityByDistance(0, 0.08, TARGET_VISION_DISTANCE, visionDistance));
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
        return (!seenEntry.getBoolean(false) && timeout.get() > TIMER_DELAY) || distanceEntry.getDouble(0) < TARGET_VISION_DISTANCE;
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
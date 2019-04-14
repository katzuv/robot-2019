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
    private double TARGET_VISION_DISTANCE = 1.12;

    private MiniPID angularVelocityPid = new MiniPID(Constants.PIDAngularVelocity[0], Constants.PIDAngularVelocity[1], Constants.PIDAngularVelocity[2]);
    private MiniPID linearVelocityPid = new MiniPID(Constants.PIDLinearVelocity[0], Constants.PIDLinearVelocity[1], Constants.PIDLinearVelocity[2]);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");

    public VisionDrive() {
        angularVelocityPid.setOutputLimits(-3, 3);
        linearVelocityPid.setOutputLimits(-1.5, 1.5);
        linearVelocityPid.setDirection(true); //reverse the direction
        requires(drivetrain);
    }

    protected void initialize() {
        updateConstants();
    }

    protected void execute() {
        double visionAngle = angleEntry.getDouble(0);
        double visionDistance = distanceEntry.getDouble(0);

        double tangentVelocity = Constants.ROBOT_WIDTH / 2.0 * angularVelocityPid.getOutput(visionAngle, 0);
        double linearVelocity = linearVelocityPid.getOutput(visionDistance, TARGET_VISION_DISTANCE);

        drivetrain.setVelocity(linearVelocity + tangentVelocity, linearVelocity - tangentVelocity);
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
        angularVelocityPid.setPID(Constants.PIDAngularVelocity[0], Constants.PIDAngularVelocity[1], Constants.PIDAngularVelocity[2]);
    }
}
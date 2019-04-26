package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;
import robot.utilities.Utils;

import static robot.Robot.drivetrain;
import static robot.Robot.isRobotA;

/**
 * Autonomously drive to a vision target.
 */
public class VelocityVisionDrive extends Command {
    private double targetDistance = 1.2; //distance from the target to stop

    private double TIMER_DELAY = 0.1;

    private MiniPID angularVelocityPid = new MiniPID(Constants.PIDAngularVelocity[0], Constants.PIDAngularVelocity[1], Constants.PIDAngularVelocity[2]);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");
    private Timer timeout = new Timer(); //this timer is meant to prevent jumps where the vision target gets lost
    private double VISION_ACCELERATION = 0.5;
    private double END_TOLERANCE = 0.2;

    public VelocityVisionDrive() {
        angularVelocityPid.setOutputLimits(-2, 2);
        requires(drivetrain);
    }

    protected void initialize() {
        updateConstants();
        timeout.reset();
    }

    protected void execute() {
        double visionAngle = angleEntry.getDouble(0);
        double visionDistance = distanceEntry.getDouble(0);

        double tangentVelocity = Constants.ROBOT_WIDTH / 2.0 * angularVelocityPid.getOutput(visionAngle, 0);
        double linearVelocity = Math.min(1.2, Utils.velocityByDistance(0, VISION_ACCELERATION, targetDistance, visionDistance));
        if (!seenEntry.getBoolean(false)) {
            if (timeout.get() == 0)
                timeout.start();
        } else {
            double velocity = linearVelocity - tangentVelocity;
            System.out.println(velocity);
            drivetrain.setVelocity(linearVelocity - tangentVelocity, linearVelocity + tangentVelocity);

            timeout.stop();
            timeout.reset();
        }
    }

    protected boolean isFinished() {
        return (!seenEntry.getBoolean(false) && timeout.get() > TIMER_DELAY) || distanceEntry.getDouble(0) < targetDistance + END_TOLERANCE;
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
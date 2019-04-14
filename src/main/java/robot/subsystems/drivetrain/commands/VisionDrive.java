package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double END_ANGLE_TOLERANCE = 0.7;
    private double STUPID_ANGLE_FIX = 0.3; // the camera wasn't centered so i added this
    private double EXTRA_PID_DISTANCE = 0.15;
    private double TIMER_DELAY = 0.1;
    private MiniPID angularVelocityPid = new MiniPID(Constants.PIDAngularVelocity[0], Constants.PIDAngularVelocity[1], Constants.PIDAngularVelocity[2]);
    private MiniPID linearVelocityPid = new MiniPID(Constants.PIDLinearVelocity[0], Constants.PIDLinearVelocity[1], Constants.PIDLinearVelocity[2]);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");
    private Timer timeout = new Timer(); //this timer is meant to prevent jumps where the vision target gets lost

    public VisionDrive() {
        angularVelocityPid.setOutputLimits(-3, 3);
        linearVelocityPid.setOutputLimits(-1.5, 1.5);
        linearVelocityPid.setDirection(true); //reverse the direction
        requires(drivetrain);
    }

    protected void initialize() {
        updateConstants();
        timeout.reset();
        if (!seenEntry.getBoolean(false))
            cancel();
    }

    protected void execute() {
        double visionAngle = angleEntry.getDouble(0) - STUPID_ANGLE_FIX;
        double visionDistance = distanceEntry.getDouble(0);

        double tangentVelocity = Constants.ROBOT_WIDTH / 2.0 * angularVelocityPid.getOutput(visionAngle, 0);
        double linearVelocity = linearVelocityPid.getOutput(visionDistance, TARGET_VISION_DISTANCE-EXTRA_PID_DISTANCE);

        drivetrain.setVelocity(linearVelocity - tangentVelocity, linearVelocity + tangentVelocity);

        if (!seenEntry.getBoolean(false)) {
            System.out.print(timeout.get());
            System.out.println(timeout.get()==0);
            if (timeout.get() == 0)
                timeout.start();
        }
        else {
            timeout.stop();
            timeout.reset();
        }
        SmartDashboard.putNumber("timeout", timeout.get());
    }

    protected boolean isFinished() {
        return (!seenEntry.getBoolean(false) && timeout.get() > TIMER_DELAY) || (distanceEntry.getDouble(0) < TARGET_VISION_DISTANCE && Math.abs(angleEntry.getDouble(0) - STUPID_ANGLE_FIX) < END_ANGLE_TOLERANCE);

        //return false;
    }

    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    protected void interrupted() {
        end();
    }

    public void updateConstants() {
        angularVelocityPid.setPID(Constants.PIDAngularVelocity[0], Constants.PIDAngularVelocity[1], Constants.PIDAngularVelocity[2]);
        linearVelocityPid.setPID(Constants.PIDLinearVelocity[0], Constants.PIDLinearVelocity[1], Constants.PIDLinearVelocity[2]);
    }
}
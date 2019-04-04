package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;
import robot.utilities.Utils;

import static robot.Robot.drivetrain;

/**
 *
 */
public class VisionArcadeDrive extends Command {
    private MiniPID turnPid = new MiniPID(Constants.PIDVisionArcade[0], Constants.PIDVisionArcade[1], Constants.PIDVisionArcade[2]);
    private MiniPID distancePid = new MiniPID(0.1, 0, 0);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");
    private double visionAngle;
    private double visionDistance;

    public VisionArcadeDrive() {
        turnPid.setOutputLimits(-1, 1);
        distancePid.setOutputLimits(-1, 1);
        requires(drivetrain);
    }

    protected void initialize() {
        updateConstants();
    }

    protected void execute() {
        visionAngle = angleEntry.getDouble(0);
        visionDistance = distanceEntry.getDouble(0);
        double distanceOutput = distancePid.getOutput(visionDistance, 0.2);
        double turnOutput = turnPid.getOutput(visionAngle, 0);
        System.out.println(distanceOutput + " | " + -turnOutput);
        double[] arcade = Utils.arcadeDrive(distanceOutput, -turnOutput, false);
        drivetrain.setSpeed(arcade[0], arcade[1]);
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
        turnPid.setPID(Constants.PIDVisionTurn[0], Constants.PIDVisionArcade[1], Constants.PIDVisionArcade[2]);
        distancePid.setPID(Constants.PIDVision[0], Constants.PIDVision[1], Constants.PIDVision[2]);
    }
}
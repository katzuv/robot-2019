package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

/**
 *
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

        double velocity = visionDistance > 0.85 ? Constants.START_VELOCITY : Constants.END_VELOCITY;

        if (visionAngle > 1)
            turn -= Constants.MIN_AIM;
        else if (visionAngle < -1)
            turn += Constants.MIN_AIM;

        drivetrain.setVelocity(velocity - turn, velocity + turn);

        SmartDashboard.putNumber("VisionDrive: Turn value", turn);
    }

    protected boolean isFinished() {
        return (visionDistance < 0.8 && Math.abs(visionAngle) < 1.5) || !seenEntry.getBoolean(true) || visionDistance < 0.5;
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
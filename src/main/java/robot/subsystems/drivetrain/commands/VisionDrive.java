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
 *
 */
public class VisionDrive extends Command {
    private MiniPID speedPid = new MiniPID(Constants.PIDVisionSpeed[0], Constants.PIDVisionSpeed[1], Constants.PIDVisionSpeed[2]);
    private MiniPID turnPid = new MiniPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);

    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");

    private double visionAngle;
    private double visionDistance;
    public VisionDrive() {
        speedPid.setOutputLimits(-Constants.PEAK_VISION_SPEED, Constants.PEAK_VISION_SPEED);
        requires(drivetrain);
    }

    protected void initialize() {
    }

    protected void execute() {
        visionAngle = angleEntry.getDouble(0);
        visionDistance = distanceEntry.getDouble(0);

        double speed = speedPid.getOutput(visionDistance, 0.6);
        double turn = turnPid.getOutput(visionAngle, 0);

        drivetrain.setSpeed(speed - turn, speed + turn);

        SmartDashboard.putNumber("VisionDrive: Turn value", turn);
        SmartDashboard.putNumber("VisionDrive: Speed", speed);
    }

    protected boolean isFinished() {
        return visionDistance < 0.4;
    }

    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}
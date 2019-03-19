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
public class AutoDrive extends Command {
    private MiniPID speedPid = new MiniPID(Constants.PIDVisionSpeed[0], Constants.PIDVisionSpeed[1], Constants.PIDVisionSpeed[2]); //TODO: move these to constants class
    private MiniPID turnPid = new MiniPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);

    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");

    private boolean finish = false;
    private Timer timer = new Timer();
    private double lastDistance = 0;
    private boolean driven = false;

    public AutoDrive() {
        speedPid.setOutputLimits(-Constants.PEAK_VISION_SPEED, Constants.PEAK_VISION_SPEED);
        requires(drivetrain);
    }

    protected void initialize() {
        finish = false;
        driven = false;
        timer.reset();
    }

    protected void execute() {
        double angle = angleEntry.getDouble(0);
        double distance = distanceEntry.getDouble(0);
        if (distance < 0.4) { //Stops in cases where the target is too close, or when the target isn't visible(in these cases the distance is 0)
            finish = true;
        }
        double speed = speedPid.getOutput(distance, 0.6);
        double turn = turnPid.getOutput(angle, 0);
        if (finish) {
            if (!driven) {
                drivetrain.driveDistance(lastDistance - 0.3);
                driven = true;
            }
        } else {
            if (angle > 1) {
                turn -= 0.05;
            } else if (angle < -1) {
                turn += 0.05;
            }
            drivetrain.setSpeed(speed - turn, speed + turn);
            lastDistance = distance;

            SmartDashboard.putNumber("AutoDrive: Turn value", turn);
            SmartDashboard.putNumber("AutoDrive: Speed", speed);
        }

    }


    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        timer.stop();
        drivetrain.setSpeed(0, 0);
    }

    protected void interrupted() {
        end();
    }
}
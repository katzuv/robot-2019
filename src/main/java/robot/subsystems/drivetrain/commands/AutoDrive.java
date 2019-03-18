package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

import static robot.Robot.drivetrain;

/**
 *
 */
public class AutoDrive extends Command {
    private MiniPID speedPid = new MiniPID(0.25, 0.0005, 1);
    private MiniPID turnPid = new MiniPID(0.01, 0, 0.5);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");

    public AutoDrive() {
        requires(drivetrain);
    }

    protected void initialize() {
    }

    protected void execute() {
        double angle = angleEntry.getDouble(0);
        double distance = distanceEntry.getDouble(0);
        double speed = speedPid.getOutput(distance, 0.4);
        double turn = turnPid.getOutput(angle, 0);
        turn = 0;
        System.out.println(speed + " | " + distance + " | " + turn);
        drivetrain.setSpeed(speed - turn, speed + turn);
    }

    protected boolean isFinished() {
        return distanceEntry.getDouble(0) == 0;
    }

    protected void end() {
        drivetrain.setSpeed(0, 0);
    }


    protected void interrupted() {
        end();
    }
}
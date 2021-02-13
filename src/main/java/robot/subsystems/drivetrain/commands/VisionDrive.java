package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;
import robot.subsystems.drivetrain.Drivetrain;
import robot.utilities.Utils;

/**
 * Autonomously drive to a vision target.
 */
public class VisionDrive extends CommandBase {
    /*
            For now have all the constants here for testing, when done move to constants file
         */
    private double targetDistance = 1.2; //distance from the target to stop

    private Drivetrain drivetrain;
    private MiniPID turnPid = new MiniPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);
    private NetworkTableEntry angleEntry = Robot.visionTable.getEntry("tape_angle");
    private NetworkTableEntry distanceEntry = Robot.visionTable.getEntry("tape_distance");
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");
    private Timer timeout = new Timer(); //this timer is meant to prevent jumps where the vision target gets lost

    private double TIMER_DELAY = 0.1;
    private double ANGLE_SETPOINT = 0;

    public VisionDrive(Drivetrain drivetrain, double targetDistance) {
        turnPid.setOutputLimits(-0.5, 0.5);
        this.drivetrain = drivetrain;
        this.targetDistance = targetDistance;
        addRequirements(drivetrain);
    }

    public VisionDrive(Drivetrain drivetrain) {
        this(drivetrain, Constants.HATCH_TARGET_DISTANCE);
    }

    @Override
    public void initialize() {
        updateConstants();
    }

    @Override
    public void execute() {
        double visionAngle = angleEntry.getDouble(0);
        double visionDistance = distanceEntry.getDouble(0);

        double speed = Math.min(Constants.VISION_SPEED, Utils.velocityByDistance(0, 0.05, targetDistance, visionDistance));
        double turn = turnPid.getOutput(visionAngle, ANGLE_SETPOINT);

        if (visionAngle > 0.8 + ANGLE_SETPOINT) {
            turn -= Constants.MIN_AIM;
        } else if (visionAngle < -0.8 + ANGLE_SETPOINT) {
            turn += Constants.MIN_AIM;
        }

        if (seenEntry.getBoolean(false)) {
            drivetrain.setSpeed(speed - turn, speed + turn);
        }
    }

    @Override
    public boolean isFinished() {
        return (!seenEntry.getBoolean(false) && timeout.get() > TIMER_DELAY) || distanceEntry.getDouble(0) < targetDistance;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }

    public void updateConstants() {
        turnPid.setPID(Constants.PIDVisionTurn[0], Constants.PIDVisionTurn[1], Constants.PIDVisionTurn[2]);
    }
}
package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;
import robot.subsystems.drivetrain.Drivetrain;

import static robot.Robot.navx;

/**
 * Rotate the Drivetrain by a given angle.
 */
public class TurnAngle extends CommandBase {
    private double angle;
    private double setpoint;
    private boolean absolute; //if the angle is relative or absolute
    private MiniPID turnPID = new MiniPID(Constants.TURNING_PID[0], Constants.TURNING_PID[1], Constants.TURNING_PID[2]);
    private Drivetrain drivetrain;

    public TurnAngle(Drivetrain drivetrain, double absoluteAngle, boolean isAbsolute) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        turnPID.setOutputLimits(Constants.TURNING_PEAK, Constants.TURNING_PEAK);
        absolute = isAbsolute;
        angle = absoluteAngle;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        if (!absolute)
            this.setpoint = drivetrain.getAngle() + angle;
        else
            this.setpoint = angle;

        if (Robot.debug) {
            SmartDashboard.putNumber("Setpoint", setpoint);
            updateConstants();
        }
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double turnSpeed = turnPID.getOutput(drivetrain.getAngle(), setpoint);
        drivetrain.setVelocity(turnSpeed, -turnSpeed);
        if (Robot.debug) {
            SmartDashboard.putNumber("Turn Error", setpoint - drivetrain.getAngle());
            SmartDashboard.putNumber("Turn speed", turnSpeed);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs(this.setpoint - navx.getAngle()) < 4;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }

    public void updateConstants() {
        turnPID.setPID(Constants.TURNING_PID[0], Constants.TURNING_PID[1], Constants.TURNING_PID[2]);
        turnPID.setOutputLimits(-Constants.TURNING_PEAK, Constants.TURNING_PEAK);
    }
}
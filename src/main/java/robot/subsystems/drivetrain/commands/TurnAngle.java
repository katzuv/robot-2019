package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;
import static robot.Robot.navx;

/**
 * Rotate the Drivetrain by a given angle.
 */
public class TurnAngle extends Command {
    private double angle;
    private double setpoint;
    private boolean absolute; //if the angle is relative or absolute
    private MiniPID turnPID = new MiniPID(Constants.TURNING_PID[0], Constants.TURNING_PID[1], Constants.TURNING_PID[2]);

    public TurnAngle(double absoluteAngle, boolean isAbsolute) {
        requires(drivetrain);
        turnPID.setOutputLimits(Constants.TURNING_PEAK, Constants.TURNING_PEAK);
        absolute = isAbsolute;
        angle = absoluteAngle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
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
    protected void execute() {
        double turnSpeed = turnPID.getOutput(drivetrain.getAngle(), setpoint);
        drivetrain.setVelocity(turnSpeed, -turnSpeed);
        if(Robot.debug) {
            SmartDashboard.putNumber("Turn Error", setpoint - drivetrain.getAngle());
            SmartDashboard.putNumber("Turn speed", turnSpeed);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(this.setpoint - navx.getAngle()) < 4;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    public void updateConstants() {
        turnPID.setPID(Constants.TURNING_PID[0], Constants.TURNING_PID[1], Constants.TURNING_PID[2]);
        turnPID.setOutputLimits(-Constants.TURNING_PEAK, Constants.TURNING_PEAK);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
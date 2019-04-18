package robot.subsystems.drivetrain.commands;

import com.stormbots.MiniPID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.subsystems.drivetrain.Constants;
import robot.utilities.Utils;

import static robot.Robot.drivetrain;
import static robot.Robot.navx;

/**
 * Rotate the Drivetrain by a given angle.
 *
 */
public class TurnAngle extends Command {
    private double angle;
    private double setpoint;
    private double arcLength;
    private boolean absolute; //if the angle is relative or absolute
    private MiniPID turninPID = new MiniPID(Constants.TURNING_PID[0], Constants.TURNING_PID[1], Constants.TURNING_PID[2]);

    public TurnAngle(double angle) {
        requires(drivetrain);
        this.angle = angle;

        absolute = false;
    }

    public TurnAngle(double absoluteAngle, boolean isAbsolute){
        requires(drivetrain);
        absolute = isAbsolute;
         angle = absoluteAngle;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
        turninPID.setOutputLimits(Constants.TURNING_PEAK,Constants.TURNING_PEAK);
        if(!absolute)
            this.setpoint = drivetrain.getAngle() + angle;
        else
            this.setpoint = angle;
        SmartDashboard.putNumber("Setpoint", setpoint);
        drivetrain.setMotorsToBrake();

        drivetrain.driveDistance(this.arcLength, -this.arcLength);
        updateConstants();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double turnSpeed = turninPID.getOutput(drivetrain.getAngle(), setpoint);
        drivetrain.setVelocity(turnSpeed, -turnSpeed);
        SmartDashboard.putNumber("Turn Error", setpoint - drivetrain.getAngle());
        SmartDashboard.putNumber("Turn speed", turnSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(this.setpoint - navx.getAngle()) < 4;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
        drivetrain.setMotorsToCoast();
    }


    public void updateConstants() {
        turninPID.setPID(Constants.TURNING_PID[0], Constants.TURNING_PID[1],Constants.TURNING_PID[2]);
        turninPID.setOutputLimits(-Constants.TURNING_PEAK,Constants.TURNING_PEAK);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
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
        if(!absolute)
            this.setpoint = navx.getAngle() + angle;
        else
            this.setpoint = angle;
        this.arcLength = (Constants.ROBOT_WIDTH/2)*(Math.toRadians(this.setpoint));
        drivetrain.setMotorsToBrake();

        drivetrain.driveDistance(this.arcLength, -this.arcLength);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(this.setpoint - navx.getAngle()) < 1;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
        drivetrain.setMotorsToCoast();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

}
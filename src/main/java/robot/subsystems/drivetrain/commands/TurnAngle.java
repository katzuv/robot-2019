package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
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
    private boolean absolute; //if the angle is relative or absolute


    public TurnAngle(double angle) {
        this.angle = angle;
        absolute = false;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    public TurnAngle(double absoluteAngle, boolean isAbsolute){
         absolute = isAbsolute;
         angle = absoluteAngle;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
        if(!absolute)
            this.setpoint = navx.getAngle() + angle;
        else
            this.setpoint = angle;
        drivetrain.setMotorsToBrake();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double direction = Math.signum(this.setpoint - navx.getAngle()); //If the target is on the robots right, turn accordingly
        double velocity = Utils.constrainedMap(Math.abs(this.setpoint - navx.getAngle()), 0, angle, 0.4, 2.5); //Have the value start at 2.5 and end at a slower speed
        drivetrain.setVelocity(direction * velocity, -direction * velocity);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(this.setpoint - navx.getAngle()) < 5;
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
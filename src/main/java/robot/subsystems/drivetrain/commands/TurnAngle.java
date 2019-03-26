package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;

import static robot.Robot.drivetrain;
import static robot.Robot.navx;

/**
 *
 */
public class TurnAngle extends Command {
    private double angle;
    private double setpoint;

    public TurnAngle(double angle) {
        this.angle = angle;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        this.setpoint = navx.getAngle() + angle;
        drivetrain.setMotorsToBrake();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double velocity = constrainedMap(Math.abs(this.setpoint - navx.getAngle()), 0, angle, 0.3, 2.5); //Have the value start at 2.5 and end at a slower speed
        drivetrain.setVelocity(velocity, -velocity);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return this.setpoint - navx.getAngle() < 2;
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

    private double map(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    private double constrainedMap(double x, double in_min, double in_max, double out_min, double out_max){
        return Math.max(out_min, Math.min(out_max, map(x, in_min, in_max, out_min, out_max)));
    }
}
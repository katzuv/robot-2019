package robot.subsystems.cargoIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.cargoIntake.CargoIntake;
import robot.subsystems.cargoIntake.Constants;

/**
 *
 */
public class WristTurn extends Command {
    private double arcLength;
    private double angle;
    private CargoIntake cargoIntake = new CargoIntake();

    public WristTurn(double angle) {
        this.angle = angle;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
        double relativeAngle = this.angle / 180;
        double circumference = 2 * Math.PI * Constants.WRIST_RADIUS;
        this.arcLength = relativeAngle * circumference;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        cargoIntake.setWristPosition(arcLength);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
package robot.subsystems.cargo_intake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.cargo_intake.Constants;

import static robot.Robot.cargoIntake;

/**
 *
 */
public class WristTurn extends Command {
    private double angle;

    public WristTurn(double angle) {
        this.angle = angle;
        requires(cargoIntake);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);

    }


    /**
     * Use the pre-defined angles to rotate the wrist.
     *
     * @param wristState an enum, holding the states. currently holds INITIAL, UP, SHOOTING, INTAKE and MAXIMAL angles
     */
    public WristTurn(Constants.WRIST_ANGLES wristState) {
        this(wristState.getValue());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        cargoIntake.setWristAngle(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        cargoIntake.setWristAngle(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(cargoIntake.getWristAngle() - angle) < 10;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        this.cancel();
    }
}
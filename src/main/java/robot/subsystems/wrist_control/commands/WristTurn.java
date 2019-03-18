package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.wrist_control.Constants;

import static robot.Robot.wristControl;

/**
 *
 */
public class WristTurn extends Command {
    private double angle;

    public WristTurn(double angle) {
        this.angle = angle;
        requires(wristControl);
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
        wristControl.setWristAngle(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        wristControl.setWristAngle(angle);
        wristControl.preventOverShoot();
        if (wristControl.getWristAngle() < 5 && angle < 3) {
            wristControl.setWristSpeed(0);
        }
        else
            wristControl.setWristAngle(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(wristControl.getWristAngle() - angle) < 2;
    }

    // Called once after isFinished returns true
    protected void end() {
        wristControl.setWristSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.wrist_control.Constants;

import static robot.Robot.wristControl;

/**
 *
 */
public class WristTurn extends Command {
    private double angle;
    private boolean usingTimer;
    private double timeout;
    private Timer timer = new Timer();
    public WristTurn(double angle) {
        this.angle = angle;
        usingTimer=true;
        this.timeout = Constants.DEFAULT_TIMEOUT;
        requires(wristControl);
    }

    /**
     * @param angle angle in degrees to turn to, in relation to the robots zeroing
     * @param timeout timeout in seconds until the command will move on. this is mainly used for drivers, use a negative number to disable.
     */
    public WristTurn(double angle, double timeout){
        this(angle);
        if(timeout < 0)
            usingTimer = false;
        else {
            this.timeout = timeout;
            usingTimer = true;
        }
    }
    /**
     * Use the pre-defined angles to rotate the wrist.
     *
     * @param wristState an enum, holding the states. currently holds INITIAL, UP, SHOOTING, INTAKE and MAXIMAL angles
     */
    public WristTurn(Constants.WRIST_ANGLES wristState) {
        this(wristState.getValue());
    }

    public WristTurn(Constants.WRIST_ANGLES wristState, double timeout){
        this(wristState.getValue(), timeout);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        timer.reset();
        timer.start();
        wristControl.setWristAngle(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        wristControl.setWristAngle(angle);
        wristControl.preventOverShoot();
        if (wristControl.getWristAngle() < 5 && angle < 3) { //TODO: this is done in the code, is it necessary?
            wristControl.setWristSpeed(0);
        }
        else
            wristControl.setWristAngle(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
        if(usingTimer)
            return timer.get() > timeout || Math.abs(wristControl.getWristAngle() - angle) < 2;
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
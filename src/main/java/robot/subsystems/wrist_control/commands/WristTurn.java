package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.subsystems.wrist_control.Constants;
import robot.subsystems.wrist_control.WristControl;

/**
 *
 */
public class WristTurn extends CommandBase {
    private WristControl wristControl;
    private double angle;
    private boolean usingTimer;
    private double timeout;
    private Timer timer = new Timer();

    public WristTurn(WristControl wristControl, double angle) {
        addRequirements(wristControl);
        this.wristControl = wristControl;
        this.angle = angle;
        usingTimer = true;
        this.timeout = Constants.DEFAULT_TIMEOUT;
    }

    /**
     * @param angle   angle in degrees to turn to, in relation to the robots zeroing
     * @param timeout timeout in seconds until the command will move on. this is mainly used for drivers, use a negative number to disable.
     */
    public WristTurn(WristControl wristControl, double angle, double timeout) {
        this(wristControl, angle);
        if (timeout < 0)
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
    public WristTurn(WristControl wristControl,Constants.WRIST_ANGLES wristState) {
        this(wristControl, wristState.getValue());
    }

    public WristTurn(WristControl wristControl,Constants.WRIST_ANGLES wristState, double timeout) {
        this(wristControl, wristState.getValue(), timeout);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        wristControl.setWristAngle(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        wristControl.setWristAngle(angle);
        wristControl.preventOverShoot();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        //If the target is in a range where the wrist should drop itself once reaching it, also allow the command to finish if the wrist is beyond the angle.
        if (angle > wristControl.getMaximalAngle() - Constants.DROP_WRIST_ANGLE) {
            if (usingTimer)
                return timer.get() > timeout || wristControl.getWristAngle() > angle - Constants.DROP_WRIST_ANGLE;
            return wristControl.getWristAngle() > angle - Constants.DROP_WRIST_ANGLE;
        } else if (angle < Constants.DROP_WRIST_ANGLE) {
            if (usingTimer)
                return timer.get() > timeout || wristControl.getWristAngle() < angle + Constants.WRIST_THRESHOLD;
            return wristControl.getWristAngle() < angle + Constants.WRIST_THRESHOLD;
        }

        if (usingTimer)
            return timer.get() > timeout || Math.abs(wristControl.getWristAngle() - angle) < Constants.WRIST_THRESHOLD;
        return Math.abs(wristControl.getWristAngle() - angle) < Constants.WRIST_THRESHOLD;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        wristControl.setWristAngle(angle);
    }
}
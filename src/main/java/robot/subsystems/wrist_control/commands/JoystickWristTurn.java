package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.OI;
import robot.Robot;
import robot.subsystems.wrist_control.Constants;

import static robot.Robot.wristControl;

/**
 *
 */
public class JoystickWristTurn extends CommandBase {
    public JoystickWristTurn() {
        addRequirements(wristControl);
    }

    // Called just before this Command runs the first time
    public void initialize() {
    }

    public void execute() {
        double yAxis = Robot.m_oi.WristStick(); // invert the input to make up positive and down negative
        if (!Robot.m_oi.enableWrist())
            return;
        // MAPPING (|dead-band to 1| -> |0 to 1|)
        yAxis -= yAxis > 0 ? OI.XBOX_JOYSTICK_DEAD_BAND : -OI.XBOX_JOYSTICK_DEAD_BAND;
        yAxis *= 1 / (1 - OI.XBOX_JOYSTICK_DEAD_BAND);
        double change;
        if (yAxis > 0)
            change = yAxis * OI.WRIST_ROTATE_RATE;
        else
            change = yAxis * OI.WRIST_ROTATE_RATE;
        wristControl.setWristAngle(Math.max(0, Math.min(wristControl.getWristAngle(), wristControl.getMaximalAngle())) + change);

    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished(boolean interrupted) {
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
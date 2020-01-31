package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.OI;
import robot.Robot;

import static robot.Robot.wristControl;

/**
 *
 */
public class JoystickRawWristTurn extends CommandBase {
    public JoystickRawWristTurn() {
        addRequirements(wristControl);
    }

    // Called just before this Command runs the first time
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        double yAxis = Robot.m_oi.WristStick(); // invert the input to make up positive and down negative
        // MAPPING (|dead-band to 1| -> |0 to 1|)
        yAxis = Math.abs(yAxis) < OI.XBOX_JOYSTICK_DEAD_BAND ? 0 : yAxis;
        yAxis *= 1 / (1 - OI.XBOX_JOYSTICK_DEAD_BAND);
        wristControl.setWristSpeed(yAxis*0.4);
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
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
package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;
import robot.subsystems.wrist_control.WristControl;

/**
 *
 */
public class JoystickWristTurn extends CommandBase {
    private WristControl wristControl;

    public JoystickWristTurn(WristControl wristControl) {
        addRequirements(wristControl);
        this.wristControl = wristControl;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double yAxis = Robot.m_oi.WristStick(); // invert the input to make up positive and down negative
        if (!Robot.m_oi.enableWrist())
            return;
        // MAPPING (|dead-band to 1| -> |0 to 1|)
        yAxis -= yAxis > 0 ? Robot.m_oi.XBOX_JOYSTICK_DEAD_BAND : -Robot.m_oi.XBOX_JOYSTICK_DEAD_BAND;
        yAxis *= 1 / (1 - Robot.m_oi.XBOX_JOYSTICK_DEAD_BAND);
        double change;
        if (yAxis > 0)
            change = yAxis * Robot.m_oi.WRIST_ROTATE_RATE;
        else
            change = yAxis * Robot.m_oi.WRIST_ROTATE_RATE;
        wristControl.setWristAngle(Math.max(0, Math.min(wristControl.getWristAngle(), wristControl.getMaximalAngle())) + change);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }
}
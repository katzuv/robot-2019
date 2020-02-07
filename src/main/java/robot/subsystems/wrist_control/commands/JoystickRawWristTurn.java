package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;
import robot.subsystems.wrist_control.WristControl;

/**
 *
 */
public class JoystickRawWristTurn extends CommandBase {
    private WristControl wristControl;

    public JoystickRawWristTurn(WristControl wristControl) {
        addRequirements(wristControl);
        this.wristControl = wristControl;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double yAxis = Robot.m_oi.WristStick(); // invert the input to make up positive and down negative
        // MAPPING (|dead-band to 1| -> |0 to 1|)
        yAxis = Math.abs(yAxis) < Robot.m_oi.XBOX_JOYSTICK_DEAD_BAND ? 0 : yAxis;
        yAxis *= 1 / (1 - Robot.m_oi.XBOX_JOYSTICK_DEAD_BAND);
        wristControl.setWristSpeed(yAxis * 0.4);
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
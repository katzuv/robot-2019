package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;

import static robot.Robot.drivetrain;
import static robot.Robot.m_oi;

/**
 *
 */
public class JoystickTurn extends CommandBase {

    public JoystickTurn() {
        addRequirements(drivetrain);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        if (m_oi.left_joystick_two.get())
            drivetrain.setSpeed(m_oi.leftStick.getY(), -m_oi.leftStick.getY());
        else
            drivetrain.setSpeed(-m_oi.rightStick.getY(), m_oi.rightStick.getY());
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    // Called when another command which requires one or more of the same
// subsystems is scheduled to run
    protected void interrupted() {
    }
}
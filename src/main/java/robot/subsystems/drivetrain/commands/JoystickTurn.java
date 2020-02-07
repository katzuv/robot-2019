package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.subsystems.drivetrain.Drivetrain;

import static robot.Robot.m_oi;

/**
 *
 */
public class JoystickTurn extends CommandBase {
    private Drivetrain drivetrain;

    public JoystickTurn(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (m_oi.left_joystick_two.get())
            drivetrain.setSpeed(m_oi.leftStick.getY(), -m_oi.leftStick.getY());
        else
            drivetrain.setSpeed(-m_oi.rightStick.getY(), m_oi.rightStick.getY());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }
}
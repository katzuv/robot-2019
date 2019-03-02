package robot.subsystems.cargo_intake.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.OI;
import robot.Robot;

/**
 *
 */
public class JoystickSpeedWristTurn extends InstantCommand {

    public JoystickSpeedWristTurn() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.cargoIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        double yAxis = - Robot.m_oi.xbox.getRawAxis(OI.right_y_stick); // invert the input to make up positive and down negative
        Robot.cargoIntake.setWristSpeed(yAxis);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
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
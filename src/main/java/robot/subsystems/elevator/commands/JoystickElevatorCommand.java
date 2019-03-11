package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.OI;
import robot.Robot;
import robot.subsystems.elevator.Elevator;
/**
 *
 */
public class JoystickElevatorCommand extends Command {

    private Elevator elevator = Robot.elevator;


    public JoystickElevatorCommand() {
        requires(elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double yAxis = -Robot.m_oi.xbox.getRawAxis(1); // invert the input to make up positive and down negative
        if (!Robot.m_oi.xbox.getRawButton(9))
            return;
        // MAPPING (|dead-band to 1| -> |0 to 1|)
        yAxis -= yAxis > 0 ? OI.XBOX_JOYSTICK_DEAD_BAND : -OI.XBOX_JOYSTICK_DEAD_BAND;
        yAxis *= 1 / (1 - OI.XBOX_JOYSTICK_DEAD_BAND);
        double change;
        if (yAxis > 0)
            change = yAxis * OI.UP_SPEED_RATE;
        else
            change = yAxis * OI.DOWN_SPEED_RATE;
        elevator.setHeight(elevator.getHeight() + change);
        elevator.moveElevator();
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
        end();
        cancel();
    }
}
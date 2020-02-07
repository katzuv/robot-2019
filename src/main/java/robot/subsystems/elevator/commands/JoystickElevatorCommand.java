package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;
import robot.subsystems.elevator.Elevator;

/**
 * this command is used to move the elevator with the joystick instead of set heights
 */
public class JoystickElevatorCommand extends CommandBase {
    private Elevator elevator;


    public JoystickElevatorCommand(Elevator elevator) {
        addRequirements(elevator);
        this.elevator = elevator;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        double yAxis = Robot.m_oi.ElevatorStick(); // invert the input to make up positive and down negative
        if (!Robot.m_oi.enableElevator())
            return;
        // MAPPING (|dead-band to 1| -> |0 to 1|)
        yAxis -= yAxis > 0 ? Robot.m_oi.XBOX_JOYSTICK_DEAD_BAND : -Robot.m_oi.XBOX_JOYSTICK_DEAD_BAND;
        yAxis *= 1 / (1 - Robot.m_oi.XBOX_JOYSTICK_DEAD_BAND);
        double change;
        if (yAxis > 0)
            change = yAxis * Robot.m_oi.UP_SPEED_RATE;
        else
            change = yAxis * Robot.m_oi.DOWN_SPEED_RATE;
        elevator.setHeight(elevator.getHeight() + change);
        elevator.moveElevator();
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
package robot.subsystems.elevator.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;
import robot.subsystems.elevator.Elevator;

/**
 * Control the raw output of the elevator using the joystick.
 */
public class JoystickElevatorSpeed extends CommandBase {
    private Elevator elevator;

    public JoystickElevatorSpeed(Elevator elevator) {
        addRequirements(elevator);
        this.elevator = elevator;
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
        double speed = 1 * Robot.m_oi.ElevatorStick();
        elevator.setSpeed(speed);
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
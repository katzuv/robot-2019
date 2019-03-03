package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

import static robot.Robot.cargoIntake;

/**
 *
 */
public class ConditionalElevatorCommand extends ConditionalCommand {

    public ConditionalElevatorCommand(Command onTrue, Command onFalse) {
        super(onTrue, onFalse);

    }


    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    @Override
    protected boolean condition() {
        return cargoIntake.isCargoInside();
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
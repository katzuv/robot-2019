package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

/**
 *this command is called when the pickupgamepiece commandgroup is called, when the robot wants to pick up the nearest game piece
 * this command is called after the robot is next to the piece and decides weather to activate the hatch intake or the cargo intake
 */
public class HatchOrCargo extends ConditionalCommand {


    public HatchOrCargo(HatchIntake onTrue, CargoIntake onFalse) {
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
        boolean pickHatch = true;//will later be smartdashboard input from image detection
        return pickHatch;
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
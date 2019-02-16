package robot.subsystems.cargoIntake.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.OI;
import robot.Robot;
import robot.subsystems.cargoIntake.CargoIntake;

import static robot.Robot.cargoIntake;

/**
 *
 */
public class GripperControl extends Command {
    private double speed;//speed of the gripper
    private boolean continuousWhileHeld;//indicator for if it should stop when the button isn't pressed or when the cargo is inside

    public GripperControl(double speed, boolean continuousWhileHeld) {
        this.speed = speed;
        this.continuousWhileHeld = continuousWhileHeld;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(cargoIntake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        cargoIntake.setGripperSpeed(speed);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (continuousWhileHeld)
            return false;
        else
            return cargoIntake.isCargoInside() || OI.y.get();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
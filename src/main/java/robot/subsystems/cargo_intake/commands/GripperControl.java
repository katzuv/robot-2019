package robot.subsystems.cargo_intake.commands;

import edu.wpi.first.wpilibj.command.Command;

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
        if (!(cargoIntake.isCargoInside() && speed < 0))
            cargoIntake.setGripperSpeed(speed);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (speed < 0)
            return !continuousWhileHeld || cargoIntake.isCargoInside();
        else
            return !continuousWhileHeld;
    }

    // Called once after isFinished returns true
    protected void end() {
        cargoIntake.setGripperSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
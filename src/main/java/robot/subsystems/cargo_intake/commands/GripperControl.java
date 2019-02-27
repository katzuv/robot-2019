package robot.subsystems.cargo_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import static robot.Robot.cargoIntake;

/**
 * Instant Command in charge of controlling the speed of the wrist wheels, the wheels in charge of grabbing and releasing cargo.
 *
 * @author Lior
 */
public class GripperControl extends InstantCommand {
    private double speed;//speed of the gripper

    public GripperControl(double speed) {
        requires(cargoIntake);
        this.speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (!(cargoIntake.isCargoInside() && speed < 0))
            cargoIntake.setGripperSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return cargoIntake.isCargoInside() && speed < 0;
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
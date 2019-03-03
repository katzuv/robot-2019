package robot.subsystems.cargo_intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.subsystems.cargo_intake.Constants.GRIPPER_SPEED;

import static robot.Robot.cargoIntake;

/**
 * Instant Command in charge of controlling the speed of the wrist wheels, the wheels in charge of grabbing and releasing cargo.
 *
 * @author Lior
 */
public class GripperControl extends Command {
    private double speed;//speed of the gripper
    private double timeout = 0;

    public GripperControl(double speed){
        requires(cargoIntake);
        this.speed = speed;
    }

    public GripperControl(GRIPPER_SPEED gripperSpeed) {
        this(gripperSpeed.getValue());
    }

    public GripperControl(GRIPPER_SPEED gripperSpeed, double timeout) {
        this(gripperSpeed);
        this.timeout = timeout;
    }

    public GripperControl(double speed, double timeout) {
        this(speed);
        this.timeout = timeout;
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
        if (speed < 0)
            return cargoIntake.isCargoInside() && speed < 0;
        else {
            return false;
        }
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
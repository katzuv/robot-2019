package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

/**
 *
 */
public class ResetEncoders extends InstantCommand {

    public ResetEncoders() {
        requires(Robot.wristControl);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.wristControl.resetSensors();
    }
}
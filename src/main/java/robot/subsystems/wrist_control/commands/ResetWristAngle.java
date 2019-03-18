package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

import static robot.Robot.wristControl;

/**
 *
 */
public class ResetWristAngle extends InstantCommand {

    public ResetWristAngle() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        wristControl.setEncoderAngle(168);
    }

}
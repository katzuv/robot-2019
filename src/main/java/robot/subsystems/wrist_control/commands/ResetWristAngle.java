package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static robot.Robot.wristControl;

/**
 *
 */
public class ResetWristAngle extends InstantCommand {
    private double angle;

    public ResetWristAngle(double angle) {
        this.angle = angle;
    }

    // Called just before this Command runs the first time
    public void initialize() {
        wristControl.setEncoderAngle(angle);
    }

}
package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

import static robot.Robot.wristControl;

/**
 *
 */
public class ResetWristAngle extends InstantCommand {
    private double angle;

    public ResetWristAngle(double angle) {
        this.angle = angle;
        setRunWhenDisabled(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        wristControl.setEncoderAngle(angle);
    }

}
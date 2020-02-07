package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import robot.subsystems.wrist_control.WristControl;

/**
 *
 */
public class ResetWristAngle extends InstantCommand {
    private WristControl wristControl;
    private double angle;

    public ResetWristAngle(WristControl wristControl, double angle) {
        addRequirements(wristControl);
        this.angle = angle;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        wristControl.setEncoderAngle(angle);
    }

}
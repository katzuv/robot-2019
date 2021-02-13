package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import robot.Robot;

/**
 * Switch the camera being streamed to the driver station.
 */
public class SwitchCamera extends InstantCommand {
    private final NetworkTableEntry streamedCameraEntry;

    public SwitchCamera() {
        this.streamedCameraEntry = Robot.visionTable.getEntry("camera");
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        String currentState = streamedCameraEntry.getString("cargo");
        streamedCameraEntry.setString(currentState.equals("cargo") ? "hatch" : "cargo");
    }
}
package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

/**
 *
 */
public class SwitchCamera extends InstantCommand {
    private final NetworkTableEntry entry;

    public SwitchCamera() {
        this.entry = Robot.visionTable.getEntry("camera");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        String currentState = entry.getString("cargo");
        entry.setString(currentState.equals("cargo") ? "hatch" : "cargo");
    }
}
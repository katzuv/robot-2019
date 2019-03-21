package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.commands.DistanceDrive;
import robot.subsystems.drivetrain.commands.VisionDrive;

/**
 * Drives to the vision target which is being sent in the dashboard.
 */
public class DriveToVisionTarget extends CommandGroup {

    public DriveToVisionTarget() {
        addSequential(new VisionDrive());
        addSequential(new DistanceDrive(-0.2));
    }
}
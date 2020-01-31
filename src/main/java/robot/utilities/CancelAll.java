package robot.utilities;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 *
 */
public class CancelAll extends InstantCommand {

    public CancelAll() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();
    }

}
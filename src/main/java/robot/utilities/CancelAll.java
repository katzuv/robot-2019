package robot.utilities;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 *
 */
public class CancelAll extends InstantCommand {

    public CancelAll() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Scheduler.getInstance().removeAll();
    }

}
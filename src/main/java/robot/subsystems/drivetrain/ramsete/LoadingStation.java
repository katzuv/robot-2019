package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.hatch_intake.commands.TakeHatch;

/**
 *
 */
public class LoadingStation extends CommandGroup {

    public LoadingStation() {
//        addSequential(new DrivePathNew(trajectory));
        addSequential(new VisionTarget(0.6));
        addSequential(new TakeHatch());
    }
}

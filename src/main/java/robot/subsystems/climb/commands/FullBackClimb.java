package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.climb.Climb;
import robot.subsystems.climb.Constants;

/**
 *
 */
public class FullBackClimb extends CommandGroup {

    public FullBackClimb() {
        addSequential(new RiseToHeight(Climb.HAB_LEG_HEIGHTS.LEVEL3));
        addSequential(new WaitCommand(5));
        addSequential(new CloseBackLegs());
        addSequential(new WaitCommand(1.5));
        addSequential(new CloseForwardLegs());
    }
}
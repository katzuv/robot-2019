package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.climb.Climb;

/**
 *
 */
public class FullForwardClimb extends CommandGroup {

    public FullForwardClimb() {

        addSequential(new RiseToHeightGyro(Climb.HAB_LEG_HEIGHTS.LEVEL3));
        addSequential(new WaitCommand(5));
        addSequential(new CloseForwardLegs());
        addSequential(new WaitCommand(1.5));
        addSequential(new CloseBackLegs());
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
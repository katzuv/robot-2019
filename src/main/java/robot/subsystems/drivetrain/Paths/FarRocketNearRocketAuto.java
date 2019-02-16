package robot.subsystems.drivetrain.Paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.Paths.Subpaths.DriveToRocket;
import robot.subsystems.drivetrain.Paths.Subpaths.FarRocketToHab;
import robot.subsystems.drivetrain.Paths.Subpaths.HabToFarRocket;
import robot.subsystems.drivetrain.Paths.Subpaths.RocketToLoading;

/**
 *
 */
public class FarRocketNearRocketAuto extends CommandGroup {

    public FarRocketNearRocketAuto() {
        addSequential(new HabToFarRocket(true));
        addSequential(new DriveToRocket());
        addSequential(new FarRocketToHab());
        addSequential(new RocketToLoading(false));
        addSequential(new RocketToLoading(true));
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
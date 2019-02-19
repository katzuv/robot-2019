package robot.subsystems.drivetrain.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.paths.subpaths.DriveToRocket;
import robot.subsystems.drivetrain.paths.subpaths.FarRocketToHab;
import robot.subsystems.drivetrain.paths.subpaths.HabToFarRocket;
import robot.subsystems.drivetrain.paths.subpaths.RocketToLoading;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Gripper;

/**
 *
 */
public class FarRocketNearRocketAuto extends CommandGroup {

    public FarRocketNearRocketAuto() {
        addSequential(new HabToFarRocket(true));// drive to far rocket
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.LEVEL1_HATCH));
        addSequential(new DriveToRocket());// drive to rocket when in his range of view
        addSequential(new Gripper(false));
        addSequential(new FarRocketToHab());// drive from far rocket to loading
        addSequential(new RocketToLoading(false));//drive to loading when in his range of view
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.LOADING_STATION));
        addSequential(new Gripper(true));
        addSequential(new RocketToLoading(true));//drive to close rocket from loading
        addSequential(new ElevatorCommand(Constants.ELEVATOR_STATES.LEVEL1_HATCH));
        addSequential(new Gripper(false));
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
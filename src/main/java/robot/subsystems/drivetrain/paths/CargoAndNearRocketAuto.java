package robot.subsystems.drivetrain.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.paths.subpaths.CargoToLoading;
import robot.subsystems.drivetrain.paths.subpaths.HabToCargo;
import robot.subsystems.drivetrain.paths.subpaths.RocketToLoading;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Gripper;
import robot.subsystems.hatch_intake.commands.PlaceHatch;

/**
 *
 */
public class CargoAndNearRocketAuto extends CommandGroup {

    public CargoAndNearRocketAuto() {
        addSequential(new HabToCargo());//drive to cargo
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.SHIP_HATCH));
        addSequential(new PlaceHatch(Constants.ELEVATOR_STATES.SHIP_HATCH));
        addSequential(new CargoToLoading());// drive to loading
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.LOADING_STATION));
        addSequential(new Gripper(true));
        addSequential(new RocketToLoading(true));// drive to rocket
        addSequential(new PlaceHatch(Constants.ELEVATOR_STATES.LEVEL1_HATCH));
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
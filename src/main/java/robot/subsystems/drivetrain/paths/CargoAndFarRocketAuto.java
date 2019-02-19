package robot.subsystems.drivetrain.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.paths.subpaths.CargoToLoading;
import robot.subsystems.drivetrain.paths.subpaths.DriveToRocket;
import robot.subsystems.drivetrain.paths.subpaths.HabToCargo;
import robot.subsystems.drivetrain.paths.subpaths.LoadingToFarRocket;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Gripper;

/**
 *
 */
public class CargoAndFarRocketAuto extends CommandGroup {

    public CargoAndFarRocketAuto() {
        addSequential(new HabToCargo());// drive to cargo ship
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.SHIP_HATCH));
        addSequential(new Gripper(false));// open gripper
        addSequential(new CargoToLoading());// drive to loading station
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.LOADING_STATION));
        addSequential(new Gripper(true));//close gripper
        addSequential(new LoadingToFarRocket(true));// drive to far rocket
        addSequential(new DriveToRocket());// drive to rocket when it is in his range of view
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
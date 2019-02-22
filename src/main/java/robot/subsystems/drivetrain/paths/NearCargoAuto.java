package robot.subsystems.drivetrain.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.paths.subpaths.CargoToLoading;
import robot.subsystems.drivetrain.paths.subpaths.HabToCargo;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Gripper;
import robot.subsystems.hatch_intake.commands.PlaceHatch;

public class NearCargoAuto extends CommandGroup {
    public NearCargoAuto() {
        addSequential(new HabToCargo());//drive to cargo
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.SHIP_HATCH));
        addSequential(new PlaceHatch());
        addSequential(new CargoToLoading());//drive to loading
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.LOADING_STATION));
        addSequential(new Gripper(true));
        addSequential(new CargoToLoading());//drive to the other cargo
        addParallel(new ElevatorCommand(Constants.ELEVATOR_STATES.SHIP_HATCH));
        addSequential(new PlaceHatch());
    }
}
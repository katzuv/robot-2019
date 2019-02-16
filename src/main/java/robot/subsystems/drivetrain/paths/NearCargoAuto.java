package robot.subsystems.drivetrain.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.paths.subpaths.CargoToLoading;
import robot.subsystems.drivetrain.paths.subpaths.HabToCargo;

public class NearCargoAuto extends CommandGroup {
    public NearCargoAuto() {
        addSequential(new HabToCargo());//drive to cargo
        addSequential(new CargoToLoading());//drive to loading
        addSequential(new CargoToLoading());//drive to the other cargo
    }
}

package robot.subsystems.drivetrain.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.paths.subpaths.CargoToLoading;
import robot.subsystems.drivetrain.paths.subpaths.HabToCargo;

public class NearCargoAuto extends CommandGroup {
    public NearCargoAuto() {
        addSequential(new HabToCargo());
        addSequential(new CargoToLoading());
        addSequential(new CargoToLoading());
        addSequential(new HabToCargo());
    }
}

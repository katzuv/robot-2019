package robot.subsystems.drivetrain.Paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.Paths.Subpaths.CargoToLoading;
import robot.subsystems.drivetrain.Paths.Subpaths.HabToCargo;

public class NearCargoAuto extends CommandGroup {
    public NearCargoAuto() {
        addSequential(new HabToCargo());
        addSequential(new CargoToLoading());
        addSequential(new CargoToLoading());
        addSequential(new HabToCargo());
    }
}

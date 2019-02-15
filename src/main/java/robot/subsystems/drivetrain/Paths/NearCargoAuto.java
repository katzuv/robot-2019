package robot.subsystems.drivetrain.Paths;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class NearCargoAuto extends CommandGroup {
    public NearCargoAuto() {
        addSequential(new HabToCargo());
        addSequential(new CargoToLoading());
        addSequential(new CargoToLoading());
        addSequential(new HabToCargo());
    }
}

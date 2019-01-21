package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class insertHatchCG extends CommandGroup {
    public insertHatchCG() {
        addSequential(new extensionCommand(true));
        addSequential(new GrabCommand(true));
        addSequential(new extensionCommand(false));
    }
}

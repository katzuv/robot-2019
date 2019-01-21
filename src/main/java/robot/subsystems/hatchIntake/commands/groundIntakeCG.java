package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class groundIntakeCG extends CommandGroup {

    public groundIntakeCG() {
        addSequential(new FoldCommand());
        addSequential(new GrabCommand(false));
    }
}

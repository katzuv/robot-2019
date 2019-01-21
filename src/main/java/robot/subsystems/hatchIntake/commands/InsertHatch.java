package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class InsertHatch extends CommandGroup {
    public InsertHatch() {
        addSequential(new ExtensionCommand(true));
        addSequential(new GrabCommand(true));
        addSequential(new ExtensionCommand(false));
        addSequential(new GrabCommand(false));
    }
}

package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class InsertHatch extends CommandGroup {
    public InsertHatch() {
        addSequential(new ExtensionCommand(true));//extend
        addSequential(new GrabCommand(true));// release hatch
        //return to previous form
        addSequential(new ExtensionCommand(false));
        addSequential(new GrabCommand(false));
    }
}

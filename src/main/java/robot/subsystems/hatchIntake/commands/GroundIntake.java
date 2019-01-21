package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GroundIntake extends CommandGroup {

    public GroundIntake() {
        addSequential(new FoldCommand());
        addSequential(new GrabCommand(false));
    }
}

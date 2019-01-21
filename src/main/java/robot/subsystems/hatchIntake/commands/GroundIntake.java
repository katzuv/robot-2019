package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GroundIntake extends CommandGroup {

    public GroundIntake() {
        addSequential(new FoldCommand());//lift hatch if inside
        addSequential(new GrabCommand(false));//put it on the flower
    }
}

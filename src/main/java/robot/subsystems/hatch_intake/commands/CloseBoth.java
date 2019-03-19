package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CloseBoth extends CommandGroup {

    public CloseBoth() {
        addParallel(new Flower(false));
        addSequential(new ExtensionPlate(false));
    }
}
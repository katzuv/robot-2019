package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * command group to instantly close the system
 */
public class CloseBoth extends CommandGroup {

    public CloseBoth() {
        addParallel(new Flower(false));
//        addSequential(new Fangs(false));
    }
}
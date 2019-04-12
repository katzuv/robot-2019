package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.hatch_intake.commands.Fangs;
import robot.subsystems.hatch_intake.commands.Flower;

/**
 * Used for when placing a hatch automatically, closes the hatch when placing
 */
public class RetractHatch extends CommandGroup {

    public RetractHatch() {
        //Return to previous form
        addSequential(new Fangs(false));

        addSequential(new WaitCommand(0.5));
        addSequential(new Flower(false));
    }
}
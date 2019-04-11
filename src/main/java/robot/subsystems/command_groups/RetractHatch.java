package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.hatch_intake.commands.Pusher;
import robot.subsystems.hatch_intake.commands.Flower;

/**
 *
 */
public class RetractHatch extends CommandGroup {

    public RetractHatch() {
        //Return to previous form
        addSequential(new Pusher(false));

        addSequential(new WaitCommand(0.5));
        addSequential(new Flower(false));
    }
}
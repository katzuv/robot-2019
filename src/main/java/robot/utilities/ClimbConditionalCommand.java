package robot.utilities;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import robot.Robot;

/**
 * Checks whether the robot IS NOT climbing.
 */
public class ClimbConditionalCommand extends ConditionalCommand {

    public ClimbConditionalCommand(Command onTrue, Command onFalse) {
        super(onTrue, onFalse);

    }
    public ClimbConditionalCommand(Command onTrue) {
        super(onTrue);

    }

    @Override
    protected boolean condition() { //Runs true if the robot IS NOT climbing
        return !Robot.climb.isClimbing();
    }


}

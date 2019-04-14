package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import robot.Robot;
import robot.subsystems.climb.Constants;

/**
 *
 */
public class SafeCloseBackLegs extends ConditionalCommand {

    public SafeCloseBackLegs() {
        super(new CloseBackLegs());
    }

    @Override
    protected boolean condition() {
        return Robot.climb.getLegFRHeight() < Constants.LEVEL_TWO_LEG_LENGTH - 0.02 &&
                Robot.climb.getLegFLHeight() < Constants.LEVEL_TWO_LEG_LENGTH - 0.02;
    }
}
package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import robot.Robot;

/**
 *
 */
public class StartButtonShift extends ConditionalCommand {

    public StartButtonShift(Command onTrue, Command onFalse) {
        super(onTrue, onFalse);
    }

    @Override
    protected boolean condition() {
        return Robot.m_oi.autoShift();
    }
}
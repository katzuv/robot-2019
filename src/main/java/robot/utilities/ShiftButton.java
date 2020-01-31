package robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

/**
 *
 */
public class ShiftButton extends CustomConditionalCommand {

    private final GenericHID joystick;
    private final int button;

    public ShiftButton(GenericHID joystick, int button, Command onTrue, Command onFalse) {
        super(onTrue, onFalse);
        this.joystick = joystick;
        this.button = button;
    }

    @Override
    protected boolean condition() {
        return joystick.getRawButton(button);
    }
}
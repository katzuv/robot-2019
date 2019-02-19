package robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

public class TriggerButton extends Button {
    private final GenericHID m_joystick;
    private final int m_axis_number;
    private final double m_starting_value;
    /**
     * Create a joystick button for triggering commands.
     *
     * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
     *                     etc)
     * @param axisNumber The button number (see {@link GenericHID#getRawButton(int) }
     */
    public TriggerButton(GenericHID joystick, int axisNumber, double startingValue) {
        m_joystick = joystick;
        m_axis_number = axisNumber;
        m_starting_value = startingValue;
    }

    /**
     * Gets the value of the joystick button.
     *
     * @return The value of the joystick button
     */
    @Override
    public boolean get() {
        return m_joystick.getRawAxis(m_axis_number) > m_starting_value;
    }
}

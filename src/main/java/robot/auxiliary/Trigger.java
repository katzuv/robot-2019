package robot.auxiliary;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;

public class Trigger extends Button {
    private XboxController m_joystick;
    private GenericHID.Hand hand;

    public Trigger(XboxController joystick, GenericHID.Hand wantedHand){
        m_joystick = joystick;
        hand = wantedHand;
    }
    public boolean get(){
        if (hand == GenericHID.Hand.kRight)
            return m_joystick.getTriggerAxis(GenericHID.Hand.kRight) > 0;
        else
            return m_joystick.getTriggerAxis(GenericHID.Hand.kLeft) > 0;
    }
}

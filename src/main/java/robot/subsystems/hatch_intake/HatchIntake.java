/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.hatch_intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robot.Robot;

/**
 * Hatch subsystem for the 2019 robot 'GENESIS'
 * the hatch subsystem uses two pistons, one which grabs the hatch and the other which extends to push the hatch onto the field pieces.
 *
 * @author paulo
 */
public class HatchIntake extends SubsystemBase {

    private final DoubleSolenoid flower = new DoubleSolenoid(1, Ports.flowerForward, Ports.flowerReverse);
    private final DoubleSolenoid fangs = new DoubleSolenoid(1, Ports.pusherForward, Ports.pusherReverse);

    public HatchIntake() {
    }

    /**
     * a command to set the flower, close it if it is already open and open it if it is already closed
     */
    public void setFlower(boolean open) {
        if (open && !Robot.m_oi.elevator.isHatchMechanismInDanger())
            flower.set(DoubleSolenoid.Value.kForward);
        else
            if(!areFangsExtended())
                flower.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * @return returns true if the flower is open and false otherwise
     */
    public boolean isFlowerOpen() {
        return flower.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * if true, extend forward
     */
    public void setFangs(boolean extend) {
        if (extend && !Robot.m_oi.elevator.isHatchMechanismInDanger() && isFlowerOpen())
            fangs.set(DoubleSolenoid.Value.kForward);
        else
            fangs.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * @return true if the fangs is extended and false otherwise
     */
    public boolean areFangsExtended() {
        return fangs.get() == DoubleSolenoid.Value.kForward;
    }



    /**
     * This is called whenever the mechanism is in danger.
     */
    public void emergencyClose() {
        setFangs(false);
        setFlower(false);
    }
}
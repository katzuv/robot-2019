/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.hatch_intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import static robot.Robot.elevator;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class HatchIntake extends Subsystem { //TODO: needs java-doc
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private final DoubleSolenoid gripper = new DoubleSolenoid(1, Ports.gripperForward, Ports.gripperReverse);
    private final DoubleSolenoid gripperPlate = new DoubleSolenoid(1, Ports.gripperPlateForward, Ports.gripperPlateReverse);


    public HatchIntake() {
    }




    /**
     * a command to set the gripper, close it if it is already open and open it if it is already closed
     */
    public void setGripper(boolean open) {
        if (open && !elevator.isSetpointInDangerZone())
            gripper.set(DoubleSolenoid.Value.kForward);
        else
            gripper.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * @return returns true if the gripper is open and false otherwise
     */
    public boolean isGripperOpen() {
        return gripper.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * if true, extend forward
     */
    public void setGripperPlate(boolean extend) {
        if (extend && !elevator.isSetpointInDangerZone())
            gripperPlate.set(DoubleSolenoid.Value.kForward);
        else
            gripperPlate.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * @return true if the gripper is extended and false otherwise
     */
    public boolean isGripperPlateExtended() {
        return gripperPlate.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * @return if the hatch is inside
     */


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void emergencyClose() {
        setGripperPlate(false);
        setGripper(false);
    }
}
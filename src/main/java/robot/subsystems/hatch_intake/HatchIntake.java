/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.hatch_intake;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class HatchIntake extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private final DoubleSolenoid groundIntake = new DoubleSolenoid(Ports.groundForward, Ports.groundReverse);
    private final DoubleSolenoid gripperPlate = new DoubleSolenoid(Ports.gripperPlateForward, Ports.gripperPlateReverse);
    private final DoubleSolenoid gripper = new DoubleSolenoid(Ports.gripperForward, Ports.gripperReverse);
    private final AnalogInput hatchSensor = new AnalogInput(Ports.proximitySensor);


    public HatchIntake() {
        hatchSensor.resetAccumulator();
    }


    /**
     * @return the voltage from the sensor
     */
    public double voltage() {
        return hatchSensor.getVoltage();
    }

    /**
     * close the ground intake
     */
    public void closeIntake() {
        groundIntake.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * open the ground intake
     */
    public void openIntake() {
        groundIntake.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * a command to set the gripper, close it if it is already open and open it if it is already closed
     */
    public void setGripper() {
        if (isGripperOpen())
            gripper.set(DoubleSolenoid.Value.kForward);
        else
            gripper.set(DoubleSolenoid.Value.kReverse);
    }
    /**
     *
     * @return returns true if the gripper is open and false otherwise
     */
    public boolean isGripperOpen() {
        return gripper.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * close the extension for the gripper
     */
    public void gripperPlateClose() {
        gripperPlate.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * open the extension for the gripper
     */
    public void gripperPlateOpen() {
        gripperPlate.set(DoubleSolenoid.Value.kForward);
    }

    /**
     *
     * @return if the hatch is inside
     */
    public boolean isHatchInside() {
        return voltage() <= Constants.HATCH_IN_VOLTAGE;
    }


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
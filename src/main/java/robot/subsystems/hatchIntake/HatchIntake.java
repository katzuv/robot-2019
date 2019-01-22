/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.hatchIntake;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class HatchIntake extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private final DoubleSolenoid grountIntake = new DoubleSolenoid(Ports.groundForward, Ports.groundReverse);
    private final AnalogInput hatchSensor = new AnalogInput(Ports.prox);
    private final DoubleSolenoid gripperExtension = new DoubleSolenoid(Ports.flowerExtensionForward, Ports.flowerExtensionReverse);
    private final DoubleSolenoid gripper = new DoubleSolenoid(Ports.flowerForward, Ports.flowerReverse);

    public HatchIntake() {
        hatchSensor.resetAccumulator();
    }

    public double voltage() {
        return hatchSensor.getVoltage();
    }

    public void closeIntake() {
        grountIntake.set(DoubleSolenoid.Value.kReverse);
    }

    public void GripperOpen() {
        grountIntake.set(DoubleSolenoid.Value.kForward);
    }

    public boolean isHatchInside() {
        return voltage() <= Constants.MIN_VOLTAGE;
    }

    public void GripperClose() {
        gripper.set(DoubleSolenoid.Value.kReverse);
    }

    public void ExtensionClose() {
        gripperExtension.set(DoubleSolenoid.Value.kReverse);
    }

    public void ExtensionOpen() {
        gripperExtension.set(DoubleSolenoid.Value.kForward);
    }

    public void openIntake() {
        gripper.set(DoubleSolenoid.Value.kForward);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.gripper_wheels;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.subsystems.gripper_wheels.Constants;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class GripperWheels extends Subsystem {
    private final AnalogInput proximitySensor = new AnalogInput(Ports.proximitySensor);
    private final VictorSPX IntakeMotor = new VictorSPX(Ports.IntakeMotor);


    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public double getProximityVoltage() {
        return proximitySensor.getVoltage();//proximitySensor.getVoltage();
    }//returns the current voltage in the proximity sensor

    public void resetProximity(){
        proximitySensor.resetAccumulator();
    }

    public boolean isCargoInside() {
        return getProximityVoltage() > Constants.CARGO_IN_VOLTAGE;
    }

    public void setGripperSpeed(double speed) {
        IntakeMotor.set(ControlMode.PercentOutput, speed);
    }


    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
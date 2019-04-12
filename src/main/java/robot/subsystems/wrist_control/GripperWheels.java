/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.wrist_control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls the wheels which grab or shoot out the cargo.
 */
public class GripperWheels extends Subsystem {
    private final AnalogInput proximitySensor = new AnalogInput(Ports.proximitySensor);
    private final VictorSPX IntakeMotor = new VictorSPX(Ports.IntakeMotor);
    
    public GripperWheels(){

        IntakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * this method returns the current voltage of the proximity sensor that is located on the wrist
     * @return
     */
    public double getProximityVoltage() {
        return !Constants.PROXIMITY_DISABLED ? proximitySensor.getVoltage() : 0;
    }

    /**
     * this method resets the proximity sensor
     */
    public void resetProximity(){
        proximitySensor.resetAccumulator();
    }

    /**
     *
     * @return whether there's a cargo inside the system or not based on the proximity sensor
     */
    public boolean isCargoInside() {
        return getProximityVoltage() > Constants.CARGO_IN_VOLTAGE;
    }

    /**
     * this method sets the speed of the gripper wheels
     * @param speed
     */
    public void setGripperSpeed(double speed) {
        IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
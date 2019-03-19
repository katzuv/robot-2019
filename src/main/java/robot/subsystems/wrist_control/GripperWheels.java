/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.wrist_control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls the wheels which grab or shoot out the cargo.
 */
public class GripperWheels extends Subsystem {
    private final AnalogInput proximitySensor = new AnalogInput(Ports.proximitySensor);
    private final VictorSPX IntakeMotor = new VictorSPX(Ports.IntakeMotor);

    public double getProximityVoltage() {
        return proximitySensor.getVoltage();
    }

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
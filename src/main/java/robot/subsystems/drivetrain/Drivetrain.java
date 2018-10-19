/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */


public class Drivetrain extends Subsystem {
    private VictorSPX leftForward = new VictorSPX(Ports.leftForwardMotor);
    private VictorSPX leftBack = new VictorSPX(Ports.leftBackMotor);
    private VictorSPX rightForward = new VictorSPX(Ports.rightForwardMotor);
    private VictorSPX rightBack = new VictorSPX(Ports.rightBackMotor);

    private Encoder leftEncoder = new Encoder(Ports.leftEncoderChannelA, Ports.leftEncoderChannelB);
    private Encoder rightEncoder = new Encoder(Ports.rightEncoderChannelA, Ports.rightEncoderChannelB);

    /**
     * Set the speed for both sides.
     * @param leftSpeed Speed for the left side
     * @param rightSpeed Speed for the right side
     */
    public void setSpeed(double leftSpeed, double rightSpeed)
    {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    /**
     *Set the speed for the right side.
     * @param speed speed for the motors of the right side
     */
    private void setRightSpeed(double speed)
    {
        rightForward.set(ControlMode.PercentOutput, speed);
        rightBack.set(ControlMode.PercentOutput, speed);
    }

    /**
     *Set the speed for the left side.
     * @param speed speed for the motors of the left side
     */
    private void setLeftSpeed(double speed)
    {
        leftForward.set(ControlMode.PercentOutput, speed);
        leftBack.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
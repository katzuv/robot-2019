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
import robot.Robot;
import robot.subsystems.drivetrain.commands.JoystickDrive;

/**
 * Add your docs here.
 */


public class Drivetrain extends Subsystem {
    private final VictorSPX leftForward = new VictorSPX(Ports.leftForwardMotor);
    private final VictorSPX leftBack = new VictorSPX(Ports.leftBackMotor);
    private final VictorSPX rightForward = new VictorSPX(Ports.rightForwardMotor);
    private final VictorSPX rightBack = new VictorSPX(Ports.rightBackMotor);

    private final Encoder leftEncoder = new Encoder(Ports.leftEncoderChannelA, Ports.leftEncoderChannelB);
    private final Encoder rightEncoder = new Encoder(Ports.rightEncoderChannelA, Ports.rightEncoderChannelB);

    public Drivetrain() {
        leftEncoder.setDistancePerPulse(Constants.PULSE_PER_DISTANCE);
        rightEncoder.setDistancePerPulse(Constants.PULSE_PER_DISTANCE);
        leftForward.setInverted(Constants.LEFT_REVERSED);
        rightForward.setInverted(Constants.RIGHT_REVERSED);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new JoystickDrive());
    }

    /**
     * Set the speed for both sides.
     *
     * @param leftSpeed  Speed for the left side
     * @param rightSpeed Speed for the right side
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    /**
     * Set the speed for the right side.
     *
     * @param speed speed for the motors of the right side
     */
    private void setRightSpeed(double speed) {
        rightForward.set(ControlMode.PercentOutput, speed);
        rightBack.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Set the speed for the left side.
     *
     * @param speed speed for the motors of the left side
     */
    private void setLeftSpeed(double speed) {
        leftForward.set(ControlMode.PercentOutput, speed);
        leftBack.set(ControlMode.PercentOutput, speed);
    }

    public double getLeftSpeed(){
        return leftEncoder.getRate();
    }

    public double getRightSpeed(){
        return rightEncoder.getRate();
    }

    /**
     * @return The distance driven on the right side of the robot since the last reset
     */
    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    /**
     * @return The distance driven on the left side of the robot since the last reset
     */
    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * returns the robot NAVX yaw angle
     * @return navx yaw angle
     */
    public double getAngle(){
        return Robot.navx.getAngle();
    }

    /**
     * returns the robot NAVX pitch angle
     * @return navx pitch angle
     */
    public double getPitch(){
        return Robot.navx.getPitch();
    }

    /**
     * returns the robot NAVX roll angle
     * @return navx roll angle
     */
    public double getRoll(){
        return Robot.navx.getRoll();
    }

}
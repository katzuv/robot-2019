/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.Robot;
import robot.subsystems.drivetrain.commands.JoystickDrive;
import robot.subsystems.drivetrain.pure_pursuit.Point;

/**
 * Add your docs here.
 */

public class Drivetrain extends Subsystem {
    private final TalonSRX leftMaster = new TalonSRX(Ports.leftMaster);
    private final VictorSPX leftSlave1 = new VictorSPX(Ports.leftSlave1);
    private final VictorSPX leftSlave2 = new VictorSPX(Ports.leftSlave2);
    private final TalonSRX rightMaster = new TalonSRX(Ports.rightMaster);
    private final VictorSPX rightSlave1 = new VictorSPX(Ports.rightSlave1);
    private final VictorSPX rightSlave2 = new VictorSPX(Ports.rightSlave2);
    public Point currentLocation = new Point(0, 0);

    public Drivetrain() {
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        leftMaster.setSensorPhase(Constants.LEFT_ENCODER_REVERSED);
        rightMaster.setSensorPhase(Constants.RIGHT_ENCODER_REVERSED);

        leftSlave1.follow(leftMaster);
        leftSlave2.follow(leftMaster);
        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);

        leftMaster.setInverted(Constants.LEFT_MASTER_REVERSED);
        leftSlave1.setInverted(Constants.LEFT_SLAVE1_REVERSED);
        leftSlave2.setInverted(Constants.LEFT_SLAVE2_REVERSED);
        rightMaster.setInverted(Constants.RIGHT_MASTER_REVERSED);
        rightSlave1.setInverted(Constants.RIGHT_SLAVE1_REVERSED);
        rightSlave2.setInverted(Constants.RIGHT_SLAVE2_REVERSED);

        leftMaster.setNeutralMode(NeutralMode.Coast);
        leftSlave1.setNeutralMode(NeutralMode.Coast);
        leftSlave2.setNeutralMode(NeutralMode.Coast);
        rightMaster.setNeutralMode(NeutralMode.Coast);
        rightSlave2.setNeutralMode(NeutralMode.Coast);
        rightSlave1.setNeutralMode(NeutralMode.Coast);

        leftMaster.config_kP(0, Constants.PIDF[0], Constants.TALON_TIMEOUT_MS);
        leftMaster.config_kI(0, Constants.PIDF[1], Constants.TALON_TIMEOUT_MS);
        leftMaster.config_kD(0, Constants.PIDF[2], Constants.TALON_TIMEOUT_MS);
        leftMaster.config_kF(0, Constants.PIDF[3], Constants.TALON_TIMEOUT_MS);
        rightMaster.config_kP(0, Constants.PIDF[0], Constants.TALON_TIMEOUT_MS);
        rightMaster.config_kI(0, Constants.PIDF[1], Constants.TALON_TIMEOUT_MS);
        rightMaster.config_kD(0, Constants.PIDF[2], Constants.TALON_TIMEOUT_MS);
        rightMaster.config_kF(0, Constants.PIDF[3], Constants.TALON_TIMEOUT_MS);
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
     * Set the speed for the left side.
     *
     * @param speed speed for the motors of the left side
     */
    private void setLeftSpeed(double speed) {
        if (speed <= 1 && speed >= -1) {
            leftMaster.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Set the speed for the right side.
     *
     * @param speed speed for the motors of the right side
     */
    private void setRightSpeed(double speed) {
        if (speed <= 1 && speed >= -1) {
            rightMaster.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * @return the speed of the left side of the Drivetrain
     */
    public double getLeftSpeed() {
        return convertTicksToDistance(leftMaster.getSelectedSensorVelocity(0));
    }

    /**
     * @return the speed of the right side of the Drivetrain
     */
    public double getRightSpeed() {
        return convertTicksToDistance(leftMaster.getSelectedSensorVelocity(0));
    }

    /**
     * @return The distance driven on the left side of the robot since the last
     *         reset
     */
    public double getLeftDistance() {
        return convertTicksToDistance(leftMaster.getSelectedSensorPosition(0));
    }

    /**
     * @return The distance driven on the right side of the robot since the last
     *         reset
     */
    public double getRightDistance() {
        return convertTicksToDistance(rightMaster.getSelectedSensorPosition(0));
    }

    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0, 0, Constants.TALON_RUNNING_TIMEOUT_MS);
        rightMaster.setSelectedSensorPosition(0, 0, Constants.TALON_RUNNING_TIMEOUT_MS);
    }

    /**
     * Convert distance in meters to ticks of the encoder.
     *
     * @param distance height in meters
     * @return ticks of the encoder
     */
    private int convertDistanceToTicks(double distance) {
        return (int) (distance * Constants.TICKS_PER_METER);
    }

    /**
     * Convert ticks of the encoder to distance in meters.
     *
     * @param ticks ticks of the encoder
     * @return height in meters
     */
    private double convertTicksToDistance(int ticks) {
        return ticks / Constants.TICKS_PER_METER;
    }

    /**
     * returns the robot NAVX yaw angle
     *
     * @return navx yaw angle
     */
    public double getAngle() {
        return Robot.navx.getAngle();
    }

    /**
     * returns the robot NAVX pitch angle
     *
     * @return navx pitch angle
     */
    public double getPitch() {
        return Robot.navx.getPitch();
    }

    /**
     * returns the robot NAVX roll angle
     *
     * @return navx roll angle
     */
    public double getRoll() {
        return Robot.navx.getRoll();
    }

    public double getYaw() {
        return Robot.navx.getYaw();
    }

    public void resetLocation() {
        currentLocation.setX(0);
        currentLocation.setY(0);
    }
}
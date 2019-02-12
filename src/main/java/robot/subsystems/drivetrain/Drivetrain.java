/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.drivetrain;

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
        leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
        leftForward.setInverted(Constants.LEFT_REVERSED);
        leftBack.setInverted(Constants.LEFT_REVERSED);
        rightForward.setInverted(Constants.RIGHT_REVERSED);
        rightBack.setInverted(Constants.RIGHT_REVERSED);
        leftSlave2.follow(leftMaster);
        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);
        leftMaster.setInverted(Constants.LEFT_MASTER_REVERSED);
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

    public double getLeftSpeed() {
        return leftEncoder.getRate();
    }

    /**
     * Set the speed for the left side.
     *
     * @param speed speed for the motors of the left side
     */
    private void setLeftSpeed(double speed) {
        if (speed <= 1 && speed >= -1) {
            leftForward.set(speed);
            leftBack.set(speed);
        }

    }

    public double getRightSpeed() {
        return rightEncoder.getRate();
    }

    /**
     * Set the speed for the right side.
     *
     * @param speed speed for the motors of the right side
     */
    private void setRightSpeed(double speed) {
        if (speed <= 1 && speed >= -1) {
            rightForward.set(speed);
            rightBack.set(speed);
        }

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
        return -leftEncoder.getDistance();
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
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
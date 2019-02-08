/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Climbing subsystem
 */
public class Climb extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private TalonSRX talonUL = new TalonSRX(Ports.upLeftMotor); //TODO: change to forward left forwrard right
    private TalonSRX talonUR = new TalonSRX(Ports.upRightMotor);
    private TalonSRX talonDL = new TalonSRX(Ports.downLeftMotor);
    private TalonSRX talonDR = new TalonSRX(Ports.downRightMotor);

    public Climb(){ //TODO: add four encoders to each of the motors just as in the elevator code.
        talonUL.setInverted(Constants.UP_LEFT_TALON_REVERSE);
        talonUR.setInverted(Constants.UP_RIGHT_TALON_REVERSE);
        talonDL.setInverted(Constants.DOWN_LEFT_TALON_REVERSE);
        talonDR.setInverted(Constants.DOWN_RIGHT_TALON_REVERSE);

        //what the motor does when not given voltage (Brake - decelerate the motor, Coast - not stop the motor)
        talonUL.setNeutralMode(NeutralMode.Brake);
        talonUR.setNeutralMode(NeutralMode.Brake);
        talonDL.setNeutralMode(NeutralMode.Brake);
        talonDR.setNeutralMode(NeutralMode.Brake);

        /* set closed loop gains in slot0 */
        talonUL.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonUL.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonUL.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonUL.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

        /* set closed loop gains in slot0 */
        //I chose opposite arms because its mostly arbitrary and it might be more accurate, we want to have the monitoring on opposite sides
        talonDR.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonDR.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonDR.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonDR.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

        /* set closed loop gains in slot0 */
        talonUR.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonUR.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonUR.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonUR.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

        /* set closed loop gains in slot0 */
        //I chose opposite arms because its mostly arbitrary and it might be more accurate, we want to have the monitoring on opposite sides
        talonDL.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonDL.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonDL.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonDL.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

        //set the pairs to follow their adjacent arms
        talonUR.follow(talonUL);
        talonDL.follow(talonDR);

        configMotorEncoder(talonUL, Constants.UP_LEFT_FORWARD_HALL_REVERSED, Constants.UP_LEFT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative);
        configMotorEncoder(talonDR, Constants.DOWN_RIGHT_FORWARD_HALL_REVERSED, Constants.DOWN_RIGHT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative);
        configMotorEncoder(talonUR, Constants.UP_RIGHT_FORWARD_HALL_REVERSED, Constants.UP_RIGHT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative);
        configMotorEncoder(talonDL, Constants.DOWN_LEFT_FORWARD_HALL_REVERSED, Constants.DOWN_LEFT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public void setTalonULHeight(double height){
        talonUL.set(ControlMode.MotionMagic, metersToTicks(height));
    }

    public void setTalonURHeight(double height){
        talonUR.set(ControlMode.MotionMagic, metersToTicks(height));
    }
    public void setTalonDLHeight(double height){
        talonDL.set(ControlMode.MotionMagic, metersToTicks(height));
    }
    public void setTalonDRHeight(double height){
        talonDR.set(ControlMode.MotionMagic, metersToTicks(height));
    }

    /**

    private void configMotorEncoder(TalonSRX motorController, boolean forwardLSReversed, boolean backwardLSReversed, FeedbackDevice feedbackDevice){
        motorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);
        motorController.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                forwardLSReversed ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIMEOUT_MS
        );
        motorController.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                backwardLSReversed ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIMEOUT_MS
        );
    }

    private int metersToTicks(double meters){
        return (int)(meters * Constants.TICKS_PER_METER);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
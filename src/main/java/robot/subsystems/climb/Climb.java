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

    private TalonSRX talonFL = new TalonSRX(Ports.forwardLeftMotor);
    private TalonSRX talonFR = new TalonSRX(Ports.forwardRightMotor);
    private TalonSRX talonBL = new TalonSRX(Ports.backLeftMotor);
    private TalonSRX talonBR = new TalonSRX(Ports.backRightMotor);

    public Climb(){ //TODO: add four encoders to each of the motors just as in the elevator code.
        talonFL.setInverted(Constants.FORWARD_LEFT_TALON_REVERSE);
        talonFR.setInverted(Constants.FORWARD_RIGHT_TALON_REVERSE);
        talonBL.setInverted(Constants.BACK_LEFT_TALON_REVERSE);
        talonBR.setInverted(Constants.BACK_RIGHT_TALON_REVERSE);

        //what the motor does when not given voltage (Brake - decelerate the motor, Coast - not stop the motor)
        talonFL.setNeutralMode(NeutralMode.Brake);
        talonFR.setNeutralMode(NeutralMode.Brake);
        talonBL.setNeutralMode(NeutralMode.Brake);
        talonBR.setNeutralMode(NeutralMode.Brake);

        /* set closed loop gains in slot0 */
        talonFL.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonFL.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonFL.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonFL.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

        /* set closed loop gains in slot0 */
        //I chose opposite arms because its mostly arbitrary and it might be more accurate, we want to have the monitoring on opposite sides
        talonBR.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonBR.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonBR.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonBR.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

        /* set closed loop gains in slot0 */
        talonFR.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonFR.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonFR.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonFR.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

        /* set closed loop gains in slot0 */
        //I chose opposite arms because its mostly arbitrary and it might be more accurate, we want to have the monitoring on opposite sides
        talonBL.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonBL.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonBL.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonBL.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

        //set the pairs to follow their adjacent arms
        talonFR.follow(talonFL);
        talonBL.follow(talonBR);

        configMotorEncoder(talonFL, Constants.FORWARD_LEFT_FORWARD_HALL_REVERSED, Constants.FORWARD_LEFT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative);
        configMotorEncoder(talonBR, Constants.BACK_RIGHT_FORWARD_HALL_REVERSED, Constants.BACK_RIGHT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative);
        configMotorEncoder(talonFR, Constants.FORWARD_RIGHT_FORWARD_HALL_REVERSED, Constants.FORWARD_RIGHT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative);
        configMotorEncoder(talonBL, Constants.BACK_LEFT_FORWARD_HALL_REVERSED, Constants.BACK_LEFT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public void setLegFLHeight(double height, double legOffset){
        talonFL.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, legOffset);
    }
    public void setLegFRHeight(double height, double legOffset){
        talonFR.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, legOffset);
    }
    public void setLegBLHeight(double height, double legOffset){
        talonBL.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, legOffset);
    }
    public void setLegBRHeight(double height, double legOffset){
        talonBR.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, legOffset);
    }

    public double getLegFLHeight(){
        return talonFL.getSelectedSensorPosition(0);
    }
    public double getLegFRHeight(){
        return talonFR.getSelectedSensorPosition(0);
    }
    public double getLegBLHeight(){
        return talonBL.getSelectedSensorPosition(0);
    }
    public double getLegBRHeight(){
        return talonBR.getSelectedSensorPosition(0);
    }

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
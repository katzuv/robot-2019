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
 * Climbing subsystem for the 2019 robot 'GENESIS'
 *
 * @author paulo
 */
public class Climb extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private TalonSRX talonFL = new TalonSRX(Ports.frontLeftMotor);
    private TalonSRX talonFR = new TalonSRX(Ports.frontRightMotor);
    private TalonSRX talonBL = new TalonSRX(Ports.backLeftMotor);
    private TalonSRX talonBR = new TalonSRX(Ports.backRightMotor);
    private final AnalogInput hatchSensor = new AnalogInput(Ports.proximitySensor);//Hatch proximity sensor

    public Climb() { //TODO: add four encoders to each of the motors just as in the elevator code.
        configMotorMovement(talonFL, Constants.FRONT_LEFT_TALON_REVERSE, NeutralMode.Brake, Constants.CLIMB_PIDFE, Constants.TALON_TIMEOUT_MS);
        configMotorMovement(talonFR, Constants.FRONT_RIGHT_TALON_REVERSE, NeutralMode.Brake, Constants.CLIMB_PIDFE, Constants.TALON_TIMEOUT_MS);
        configMotorMovement(talonBL, Constants.BACK_LEFT_TALON_REVERSE, NeutralMode.Brake, Constants.CLIMB_PIDFE, Constants.TALON_TIMEOUT_MS);
        configMotorMovement(talonBR, Constants.BACK_RIGHT_TALON_REVERSE, NeutralMode.Brake, Constants.CLIMB_PIDFE, Constants.TALON_TIMEOUT_MS);

        configMotorSensors(talonFL, Constants.FRONT_LEFT_FORWARD_HALL_REVERSED, Constants.FRONT_LEFT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.TALON_TIMEOUT_MS);
        configMotorSensors(talonFR, Constants.FRONT_RIGHT_FORWARD_HALL_REVERSED, Constants.FRONT_RIGHT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.TALON_TIMEOUT_MS);
        configMotorSensors(talonBL, Constants.BACK_LEFT_FORWARD_HALL_REVERSED, Constants.BACK_LEFT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.TALON_TIMEOUT_MS);
        configMotorSensors(talonBR, Constants.BACK_RIGHT_FORWARD_HALL_REVERSED, Constants.BACK_RIGHT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.TALON_TIMEOUT_MS);
    }

    /**
     * Set the target height of the front left leg in meters.
     *
     * @param height    target height in meters.
     * @param legOffset the error of the leg from its ideal length. set to 0 if no correction is needed.
     */
    public void setLegFLHeight(double height, double legOffset) {//TODO: currently when the robot starts to tip, half the legs speed up, and the other half slow down. maybe we can set only two to slow down ect.
        talonFL.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * legOffset);
    }

    /**
     * Set the target height of the front right leg in meters.
     *
     * @param height    target height in meters.
     * @param legOffset the error of the leg from its ideal length. set to 0 if no correction is needed.
     */
    public void setLegFRHeight(double height, double legOffset) {
        talonFR.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * legOffset);
    }

    /**
     * Set the target height of the back left leg in meters.
     *
     * @param height    target height in meters.
     * @param legOffset the error of the leg from its ideal length. set to 0 if no correction is needed.
     */
    public void setLegBLHeight(double height, double legOffset) {
        talonBL.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * legOffset);
    }

    /**
     * @return height of the back right leg in meters
     */
    public void setLegBRHeight(double height, double legOffset) {
        talonBR.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * legOffset);
    }

    /**
     * @return height of the front left leg in meters
     */
    public double getLegFLHeight() {
        return ticksToMeters(talonFL.getSelectedSensorPosition(0));
    }

    /**
     * @return height of the front right leg in meters
     */
    public double getLegFRHeight() {
        return ticksToMeters(talonFR.getSelectedSensorPosition(0));
    }

    /**
     * @return height of the back left leg in meters
     */
    public double getLegBLHeight() {
        return ticksToMeters(talonBL.getSelectedSensorPosition(0));
    }

    /**
     * get the height of the leg in meters from the encoders.
     *
     * @return height of the leg in meters
     */
    public double getLegBRHeight() {
        return ticksToMeters(talonBR.getSelectedSensorPosition(0));
    }

    /**
     * auxiliary method used to shorten the code when configuring the motor controllers (the talons).
     *
     * @param motorController    the motor controller being configured
     * @param forwardLSReversed  is the forward limit switch reversed
     * @param backwardLSReversed is the forward limit switch reversed
     * @param feedbackDevice     the encoder type connected to the motor controller
     */
    private void configMotorSensors(TalonSRX motorController, boolean forwardLSReversed, boolean backwardLSReversed, FeedbackDevice feedbackDevice, int timeout) {
        motorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeout);
        motorController.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                forwardLSReversed ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                timeout
        );
        motorController.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector, backwardLSReversed
                        ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                timeout
        );
        motorController.configSelectedFeedbackSensor(feedbackDevice, 0, timeout);
    }

    /**
     * auxiliary method used to shorten the code when configuring the motor controllers (the talons).
     *
     * @param motorController the motor controller being configured.
     * @param motorInverted   whether the motor should be inverted or not.
     * @param neutralMode     neutral mode of the motor. can be either COAST or BREAK.
     * @param pidfConstants   PIDF movement constants.
     * @param timeout         Timeout when configuring the controller
     */
    private void configMotorMovement(TalonSRX motorController, boolean motorInverted, NeutralMode neutralMode, double[] pidfConstants, int timeout) {
        motorController.setInverted(motorInverted);
        motorController.setNeutralMode(neutralMode); //what the motor does when not given voltage (Brake - decelerate the motor, Coast - not stop the motor)
        motorController.config_kP(0, pidfConstants[0], timeout);
        motorController.config_kI(0, pidfConstants[1], timeout);
        motorController.config_kD(0, pidfConstants[2], timeout);
        motorController.config_kF(0, pidfConstants[3], timeout);
    }

    /**
     * auxiliary method used to make the code more understandable.
     *
     * @param meters size or length in meters
     * @return the ticks the motor needs to do.
     */
    private int metersToTicks(double meters) {
        return (int) (meters * Constants.TICKS_PER_METER);
    }

    /**
     * auxiliary method used to make the code more understandable.
     *
     * @param ticks amount of encoder ticks.
     * @return the respective height in meters.
     */
    private double ticksToMeters(int ticks) {
        return ticks / Constants.TICKS_PER_METER;
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public enum HAB_LEG_HEIGHTS { //TODO:refactor all these values.
        GROUND(0.05),
        LEVEL2(Constants.LEVEL_TWO_LEG_LENGTH),
        LEVEL3(Constants.LEVEL_THREE_LEG_LENGTH);

        private double habHeight;

        HAB_LEG_HEIGHTS(double height) {
            habHeight = height;
        }

        public double getHABHHeight() {
            return habHeight;
        }
    }

    public double getVoltage(){
        return hatchSensor.getVoltage();
    }
}
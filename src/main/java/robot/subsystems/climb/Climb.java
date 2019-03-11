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
public class Climb extends Subsystem { //TODO: only work last 30 seconds
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private TalonSRX talonFL = new TalonSRX(Ports.frontLeftMotor);
    private TalonSRX talonFR = new TalonSRX(Ports.frontRightMotor);
    private TalonSRX talonBL = new TalonSRX(Ports.backLeftMotor);
    private TalonSRX talonBR = new TalonSRX(Ports.backRightMotor);
    private TalonSRX wheelDrive = new TalonSRX(Ports.wheelMotor);

    public Climb() {
        wheelDrive.setInverted(Constants.WHEEL_TALON_REVERSE);
        configMotorMovement(talonFL, Constants.FRONT_LEFT_TALON_REVERSE, NeutralMode.Brake, Constants.CLIMB_PIDFE, Constants.MOTION_MAGIC_CRUISE_VELOCITY, Constants.MOTION_MAGIC_ACCELERATION, Constants.TALON_TIMEOUT_MS);
        configMotorMovement(talonFR, Constants.FRONT_RIGHT_TALON_REVERSE, NeutralMode.Brake, Constants.CLIMB_PIDFE, Constants.MOTION_MAGIC_CRUISE_VELOCITY, Constants.MOTION_MAGIC_ACCELERATION, Constants.TALON_TIMEOUT_MS);
        configMotorMovement(talonBL, Constants.BACK_LEFT_TALON_REVERSE, NeutralMode.Brake, Constants.CLIMB_PIDFE, Constants.MOTION_MAGIC_CRUISE_VELOCITY, Constants.MOTION_MAGIC_ACCELERATION, Constants.TALON_TIMEOUT_MS);
        configMotorMovement(talonBR, Constants.BACK_RIGHT_TALON_REVERSE, NeutralMode.Brake, Constants.CLIMB_PIDFE, Constants.MOTION_MAGIC_CRUISE_VELOCITY, Constants.MOTION_MAGIC_ACCELERATION, Constants.TALON_TIMEOUT_MS);

        configMotorSensors(talonFL, Constants.FRONT_LEFT_FORWARD_HALL_REVERSED, Constants.FRONT_LEFT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.FRONT_LEFT_ENCODER_REVERSE, Constants.TALON_TIMEOUT_MS);
        configMotorSensors(talonFR, Constants.FRONT_RIGHT_FORWARD_HALL_REVERSED, Constants.FRONT_RIGHT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.FRONT_RIGHT_ENCODER_REVERSE, Constants.TALON_TIMEOUT_MS);
        configMotorSensors(talonBL, Constants.BACK_LEFT_FORWARD_HALL_REVERSED, Constants.BACK_LEFT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.BACK_LEFT_ENCODER_REVERSE, Constants.TALON_TIMEOUT_MS);
        configMotorSensors(talonBR, Constants.BACK_RIGHT_FORWARD_HALL_REVERSED, Constants.BACK_RIGHT_REVERSE_HALL_REVERSED, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.BACK_RIGHT_ENCODER_REVERSE, Constants.TALON_TIMEOUT_MS);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    /**
     * Set the target height of the front legs in meters.
     * this method takes into consideration the error between both legs,
     * and lowers the speeds accordingly.
     *
     * @param height              target height in meters.
     * @param additionalLegOffset the error of the leg from its ideal length. set to 0 if no correction is needed.
     */
    public void setLegDriveHeight(double height, double additionalLegOffset) {
        if (isCompromised())
            return;

        talonFL.set(ControlMode.MotionMagic, metersToTicks(height),
                DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * additionalLegOffset + getLegFRHeight() - getLegFLHeight());
        talonFR.set(ControlMode.MotionMagic, metersToTicks(height),
                DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * additionalLegOffset + getLegFLHeight() - getLegFRHeight());
    }

    /**
     * Set the target height of the front legs in meters.
     * this method doesn't take into consideration the error between both front legs,
     * and should be used primarily at times where the legs aren't calibrated.
     *
     * @param height    target height in meters.
     * @param legOffset the error of the leg from its ideal length. set to 0 if no correction is needed.
     */
    public void setLegDriveHeightWithoutChecking(double height, double legOffset) {
        talonFL.set(ControlMode.MotionMagic, metersToTicks(height),
                DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * legOffset);
        talonFR.set(ControlMode.MotionMagic, metersToTicks(height),
                DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * legOffset);

    }

    public void setBackLegHeights(double height, double additionalLegOffset) {
        if(!isCompromised()){
            talonBL.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * additionalLegOffset + getLegFRHeight() - getLegFLHeight());
            talonBR.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * additionalLegOffset + getLegFRHeight() - getLegFLHeight());
        }
    }

    public void setBackLegHeightsOpenLoop(double height, double legOffset) {
        if(!isCompromised()){
            talonBL.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * legOffset);
            talonBR.set(ControlMode.MotionMagic, metersToTicks(height), DemandType.ArbitraryFeedForward, Constants.CLIMB_PIDFE[4] * legOffset);
        }
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
     * get the height of the leg in meters from the encoders
     *
     * @return height of the leg in meters
     */
    public double getLegBRHeight() {
        return ticksToMeters(talonBR.getSelectedSensorPosition(0));
    }

    /**
     * set the speed to move the legs.
     *
     * @param speed percent output from -1 to 1
     */
    public void setLegDriveSpeed(double speed) {
        if (!isCompromised()) {
            talonFL.set(ControlMode.PercentOutput, speed);
            talonFR.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * set the speed of the back legs
     *
     * @param speed percent output from -1 to 1
     */
    public void setBackLegSpeeds(double speed) {
        if (!isCompromised()) {
            talonBR.set(ControlMode.PercentOutput, speed);
            talonBL.set(ControlMode.PercentOutput, speed);

        }
    }

    public void setWheelSpeed(double speed) {
        if (!isCompromised())
            wheelDrive.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Check if all four legs are up, using the limit switches
     *
     * @return if all the legs touch their limit switches, return true.
     */
    public boolean areAllLegsUp() {
        return talonBL.getSensorCollection().isRevLimitSwitchClosed() == !Constants.BACK_LEFT_REVERSE_HALL_REVERSED
                && talonFL.getSensorCollection().isRevLimitSwitchClosed() == !Constants.FRONT_LEFT_REVERSE_HALL_REVERSED
                && talonBR.getSensorCollection().isRevLimitSwitchClosed() == !Constants.BACK_RIGHT_REVERSE_HALL_REVERSED
                && talonFR.getSensorCollection().isRevLimitSwitchClosed() == !Constants.FRONT_RIGHT_REVERSE_HALL_REVERSED;
    }

    /**
     * Checks if a talon sensor collection has disconnected, or any other extreme situation has happened which should prevent the moto motors from moving
     * <p>calls {@link #isCompromised() isCompromised}, and if true, calls {@link #emergencyStop() emergencyStop}</p>
     */
    public void executePreventBreak() {
        if (isCompromised()) {
            emergencyStop();
        }
    }

    /**
     * checks whether the climbing mechanism is in an illegal state.
     *
     * <p>
     * this includes situations where the sensors disconnect from the talon,
     * or if the height between both legs goes above a certain number.
     * do note, that this method takes into consideration that all limit switches are normally closed,
     * otherwise, this method has no way of knowing if they are working or not.
     * </p>
     *
     * @return returns whether the mechanism should be disabled. if the subsystem is fine, returns false.
     */
    public boolean isCompromised() {
        return ((!talonFL.getSensorCollection().isFwdLimitSwitchClosed() && !talonFL.getSensorCollection().isFwdLimitSwitchClosed()) ||
                (!talonFR.getSensorCollection().isFwdLimitSwitchClosed() && !talonFR.getSensorCollection().isFwdLimitSwitchClosed()) ||
                (!talonBL.getSensorCollection().isFwdLimitSwitchClosed() && !talonBL.getSensorCollection().isFwdLimitSwitchClosed()) ||
                (!talonBR.getSensorCollection().isFwdLimitSwitchClosed() && !talonBR.getSensorCollection().isFwdLimitSwitchClosed())) ||
                Math.abs(getLegFRHeight() - getLegFLHeight()) > Constants.LEG_EMERGENCY_STOP;
    }

    /**
     * disable all motor values to 0
     */
    public void emergencyStop() {
        talonFL.set(ControlMode.PercentOutput, 0);
        talonFR.set(ControlMode.PercentOutput, 0);
        talonBL.set(ControlMode.PercentOutput, 0);
        talonBR.set(ControlMode.PercentOutput, 0);
        wheelDrive.set(ControlMode.PercentOutput, 0);
    }

    /**
     * reset all four encoder values
     */
    public void resetEncoders() {
        talonFL.setSelectedSensorPosition(Constants.FRONT_LEFT_STARTING_OFFSET);
        talonFR.setSelectedSensorPosition(Constants.FRONT_RIGHT_STARTING_OFFSET);
        talonBL.setSelectedSensorPosition(Constants.BACK_LEFT_STARTING_OFFSET);
        talonBR.setSelectedSensorPosition(Constants.BACK_RIGHT_STARTING_OFFSET);
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

    /**
     * auxiliary method used to shorten the code when configuring the motor controllers (the talons).
     *
     * @param motorController    the motor controller being configured
     * @param forwardLSReversed  is the forward limit switch reversed
     * @param backwardLSReversed is the forward limit switch reversed
     * @param feedbackDevice     the encoder type connected to the motor controller
     */
    private void configMotorSensors(TalonSRX motorController, boolean forwardLSReversed, boolean backwardLSReversed, FeedbackDevice feedbackDevice, boolean encoderReversed, int timeout) {
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
        motorController.setSensorPhase(encoderReversed);
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
    private void configMotorMovement(TalonSRX motorController, boolean motorInverted, NeutralMode neutralMode, double[] pidfConstants, int cruise, int acceleration, int timeout) {
        motorController.setInverted(motorInverted);
        motorController.setNeutralMode(neutralMode); //what the motor does when not given voltage (Brake - decelerate the motor, Coast - not stop the motor)
        motorController.config_kP(0, pidfConstants[0], timeout);
        motorController.config_kI(0, pidfConstants[1], timeout);
        motorController.config_kD(0, pidfConstants[2], timeout);
        motorController.config_kF(0, pidfConstants[3], timeout);
        motorController.configMotionCruiseVelocity(cruise);
        motorController.configMotionAcceleration(acceleration);
    }

    //TODO: move this enum to the Constants class.
    public enum HAB_LEG_HEIGHTS { //TODO:refactor all these values.
        GROUND(0.05),
        TEST(0.1),
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
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.subsystems.elevator.commands.JoystickElevatorCommand;

/**
 * Elevator subsystem for the 2019 robot 'GENESIS'
 *
 * @author paulo
 */
public class Elevator extends Subsystem {

    /* pid slots for the four states: up and down on the first level and up and down on the second level of the cascade*/
    private final int TALON_LOW_UP_PID_SLOT = 0;
    private final VictorSPX victorMotor = new VictorSPX(Ports.victorPort);
    private final TalonSRX talonMotor = new TalonSRX(Ports.talonPort);
    private double setpoint;

    public Elevator() {
        talonMotor.setInverted(Constants.TALON_REVERSE);
        victorMotor.setInverted(Constants.VICTOR_REVERSE);

        //what the motor does when not given voltage (Brake - decelerate the motor, Coast - not stop the motor)
        victorMotor.setNeutralMode(NeutralMode.Brake);
        talonMotor.setNeutralMode(NeutralMode.Brake);

        /* set closed loop gains in slot0 */
        talonMotor.config_kP(TALON_LOW_UP_PID_SLOT, Constants.LIFT_LOW_UP_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kI(TALON_LOW_UP_PID_SLOT, Constants.LIFT_LOW_UP_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kD(TALON_LOW_UP_PID_SLOT, Constants.LIFT_LOW_UP_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kF(TALON_LOW_UP_PID_SLOT, Constants.LIFT_LOW_UP_PIDF[3], Constants.TALON_TIMEOUT_MS);

        talonMotor.configMotionCruiseVelocity(Constants.MOTION_MAGIC_CRUISE_SPEED);
        talonMotor.configMotionAcceleration(Constants.MOTION_MAGIC_ACCELERATION);
        //the victor follows all the inputs the talon has
        victorMotor.follow(talonMotor);

        /* Nominal Output- The "minimal" or "weakest" motor output allowed if the output is nonzero
         * Peak Output- The "maximal" or "strongest" motor output allowed.
         * These settings are useful to reduce the maximum velocity of the mechanism,
         * and can make tuning the closed-loop simpler.  */
        talonMotor.configNominalOutputForward(Constants.NOMINAL_OUT_FWD, Constants.TALON_TIMEOUT_MS);
        talonMotor.configPeakOutputForward(Constants.PEAK_OUT_FWD, Constants.TALON_TIMEOUT_MS);
        talonMotor.configNominalOutputReverse(Constants.NOMINAL_OUT_REV, Constants.TALON_TIMEOUT_MS);
        talonMotor.configPeakOutputReverse(Constants.PEAK_OUT_REV, Constants.TALON_TIMEOUT_MS);

        talonMotor.setSensorPhase(Constants.ENCODER_REVERSED);
        /* Configure the encoder */
        talonMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);

        /* Configure the hall effect sensors */
        // TOP hall effect
        talonMotor.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                Constants.TOP_HALL_REVERSED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIMEOUT_MS
        );
        // BOTTOM hall effect
        talonMotor.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                Constants.BOTTOM_HALL_REVERSED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIMEOUT_MS
        );
        talonMotor.overrideLimitSwitchesEnable(false);
        talonMotor.overrideSoftLimitsEnable(false);
    }

    /**
     * Get the target height from the encoders.
     *
     * @return Current height of the elevator in meters
     */
    public double getHeight() {
        return convertTicksToHeight(talonMotor.getSelectedSensorPosition(0));
    }

    /**
     * Set a target height and tell the motors to move to that position.
     *
     * @param height Target height of the elevator in meters
     */
    public void setHeight(double height) {
        this.setpoint = convertHeightToTicks(Math.min(Constants.ELEVATOR_MAX_HEIGHT, Math.max(0, height)));
    }

    public double getTicks() {
        return talonMotor.getSelectedSensorPosition(0);
    }

    /**
     * Needs to be run in a loop, Moves the motors and prevents overshooting using the limit switches.
     */
    public void update() {
        preventOverShoot();
        moveElevator();
    }

    /**
     * Moves the elevator to the current setpoint, assigned in setHeight()
     */
    public void moveElevator() {
        if (getHeight() < 0.05)
            talonMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, 0);
        else if (atSecondStage())
            talonMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, Constants.SECOND_STAGE_FEEDFORWARD);
        else
            talonMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, Constants.FIRST_STAGE_FEEDFORWARD);
    }

    /**
     * Beyond preventing the motors from going above a certain height, this method prevents them from moving higher or
     * lower once one of the limit switches/hall effects is pressed.
     */
    private void preventOverShoot() { //TODO: add manual override?
        if (atTop()) {
            //setHeight(Math.min(getHeight(), convertTicksToMeters(setpoint)));
            talonMotor.setSelectedSensorPosition((int) (Constants.ELEVATOR_MAX_HEIGHT * Constants.TICKS_PER_METER), 0, Constants.TALON_RUNNING_TIMEOUT_MS); //set the position to the top.
        }
        if (atBottom()) {
            //setHeight(Math.max(getHeight(), convertTicksToMeters(setpoint)));
            talonMotor.setSelectedSensorPosition(0, 0, Constants.TALON_RUNNING_TIMEOUT_MS); //set the encoder position to the bottom
        } //TODO: raise exception when both limit switches are pressed
    }

    /**
     * Get the velocity from the encoders
     *
     * @return velocity of the motor, in m/s
     */
    public double getSpeed() {
        return convertTicksToHeight(10 * talonMotor.getSelectedSensorVelocity(0)); //sensorVelocity calculates for 100ms
    }

    /**
     * Set the motor to a certain speed, on a scale of -1 to 1.
     *
     * @param speed speed of the motor from -1 to 1
     */
    public void setSpeed(double speed) {
        talonMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, Constants.FIRST_STAGE_FEEDFORWARD);
    }

    /**
     * Check if the limit switch at the top of the elevator is pressed
     *
     * @return boolean of the sensor true or false
     */
    public boolean atTop() {
        //return Math.abs(getHeight()) > Constants.ELEVATOR_MAX_HEIGHT;
        return talonMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    /**
     * Check if the limit switch at the bottom of the elevator is pressed
     *
     * @return boolean of the sensor true or false
     */
    public boolean atBottom() {
        //return Math.abs(getHeight()) < 0.05;
        return talonMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    /**
     * Check if the elevator is above its first stage.
     * Using this method we can see how much weight is being put on the elevator
     * (because of the way continuous elevators are built)
     * This method estimates what stage the elevator is at based on the encoder.
     * Not to be confused with the spaceship or HAB zone stages.
     *
     * @return stage of the robot. true being on its high part.
     */
    public boolean atSecondStage() {
        return Constants.ELEVATOR_MID_HEIGHT < getHeight();
    }

    /**
     * Get the speed of the motor in percentages
     *
     * @return return motor value, from -1 to 1
     */
    public double getOutputPrecent() {
        return talonMotor.getMotorOutputPercent();
    }

    /**
     * Convert height in meters to ticks of the encoder.
     *
     * @param height height in meters
     * @return ticks of the encoder
     */
    private int convertHeightToTicks(double height) {
        return (int) (height * Constants.TICKS_PER_METER);
    }

    /**
     * Convert ticks of the encoder to height in meters.
     *
     * @param ticks ticks of the encoder
     * @return height in meters
     */
    private double convertTicksToHeight(int ticks) {
        return ticks / Constants.TICKS_PER_METER;
    }

    public void resetEncoders() {
        talonMotor.setSelectedSensorPosition(-800, 0, Constants.TALON_RUNNING_TIMEOUT_MS);
    }

    @Override
    public void initDefaultCommand() {
        //setDefaultCommand(new ElevatorCommand(1.2));
        setDefaultCommand(new JoystickElevatorCommand());
    }
}
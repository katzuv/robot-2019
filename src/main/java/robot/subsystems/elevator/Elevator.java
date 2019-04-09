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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.elevator.commands.JoystickElevatorCommand;
import robot.subsystems.hatch_intake.HatchIntake;

/**
 * Elevator subsystem for the 2019 robot 'GENESIS'
 *
 * @author paulo
 */
public class Elevator extends Subsystem {

    /* pid slots for the four states: up and down on the first level and up and down on the second level of the cascade*/
    private final int TALON_LOW_UP_PID_SLOT = 0;
    private final VictorSPX slaveMotor = new VictorSPX(Ports.victorPort);
    private final TalonSRX masterMotor = new TalonSRX(Ports.talonPort);
    private int setpoint;

    public Elevator() {
        masterMotor.setInverted(Constants.TALON_REVERSE);
        slaveMotor.setInverted(Constants.VICTOR_REVERSE);

        //what the motor does when not given voltage (Brake - decelerate the motor, Coast - not stop the motor)
        slaveMotor.setNeutralMode(NeutralMode.Brake);
        masterMotor.setNeutralMode(NeutralMode.Brake);

        /* set closed loop gains in slot0 */
        masterMotor.config_kP(TALON_LOW_UP_PID_SLOT, Constants.LIFT_LOW_UP_PIDF[0], Constants.TALON_TIMEOUT_MS);
        masterMotor.config_kI(TALON_LOW_UP_PID_SLOT, Constants.LIFT_LOW_UP_PIDF[1], Constants.TALON_TIMEOUT_MS);
        masterMotor.config_kD(TALON_LOW_UP_PID_SLOT, Constants.LIFT_LOW_UP_PIDF[2], Constants.TALON_TIMEOUT_MS);
        masterMotor.config_kF(TALON_LOW_UP_PID_SLOT, Constants.LIFT_LOW_UP_PIDF[3], Constants.TALON_TIMEOUT_MS);

        masterMotor.configMotionCruiseVelocity(Constants.MOTION_MAGIC_CRUISE_SPEED);
        masterMotor.configMotionAcceleration(Constants.MOTION_MAGIC_ACCELERATION);
        //the victor follows all the inputs the talon has
        slaveMotor.follow(masterMotor);

        /* Nominal Output- The "minimal" or "weakest" motor output allowed if the output is nonzero
         * Peak Output- The "maximal" or "strongest" motor output allowed.
         * These settings are useful to reduce the maximum velocity of the mechanism,
         * and can make tuning the closed-loop simpler.  */
        masterMotor.configNominalOutputForward(Constants.NOMINAL_OUT_FWD, Constants.TALON_TIMEOUT_MS);
        masterMotor.configPeakOutputForward(Constants.PEAK_OUT_FWD, Constants.TALON_TIMEOUT_MS);
        masterMotor.configNominalOutputReverse(Constants.NOMINAL_OUT_REV, Constants.TALON_TIMEOUT_MS);
        masterMotor.configPeakOutputReverse(Constants.PEAK_OUT_REV, Constants.TALON_TIMEOUT_MS);

        masterMotor.setSensorPhase(Constants.ENCODER_REVERSED);
        /* Configure the encoder */
        masterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.TALON_TIMEOUT_MS);

        /* Configure the hall effect sensors */
        // TOP hall effect
        masterMotor.configForwardLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                Constants.TOP_HALL_REVERSED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIMEOUT_MS
        );
        // BOTTOM hall effect
        masterMotor.configReverseLimitSwitchSource(
                LimitSwitchSource.FeedbackConnector,
                Constants.BOTTOM_HALL_REVERSED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIMEOUT_MS
        );
        masterMotor.overrideLimitSwitchesEnable(false);
        masterMotor.overrideSoftLimitsEnable(false);

        masterMotor.configVoltageCompSaturation(12);
        masterMotor.enableVoltageCompensation(true);
    }

    /**
     * Get the target height from the encoders.
     *
     * @return Current height of the elevator in meters
     */
    public double getHeight() {
        return convertTicksToHeight(masterMotor.getSelectedSensorPosition(0));
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
        return masterMotor.getSelectedSensorPosition(0);
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
        SmartDashboard.putNumber("setPoint ", setpoint);
    if (getHeight() < Constants.ELEVATOR_HOLD_IN_PLACE_HEIGHT && setpoint < Constants.ELEVATOR_HOLD_IN_PLACE_HEIGHT) //let the robot go if its below a certain height
            masterMotor.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, Constants.FLOOR_FEEDFORWARD);
        else if (atSecondStage())
            masterMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, Constants.SECOND_STAGE_FEEDFORWARD);
        else
            masterMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, Constants.FIRST_STAGE_FEEDFORWARD);
        if (isSetpointInDangerZone())
            Robot.hatchIntake.emergencyClose();

    }

    /**
     * Beyond preventing the motors from going above a certain height, this method prevents them from moving higher or
     * lower once one of the limit switches/hall effects is pressed.
     */
    private void preventOverShoot() {
        if (atTop()) {
            //setHeight(Math.min(getHeight(), convertTicksToMeters(setpoint)));
            masterMotor.setSelectedSensorPosition((int) (Constants.ELEVATOR_MAX_HEIGHT * Constants.TICKS_PER_METER), 0, Constants.TALON_RUNNING_TIMEOUT_MS); //set the position to the top.
        }
        if (atBottom()) {
            //setHeight(Math.max(getHeight(), convertTicksToMeters(setpoint)));
            masterMotor.setSelectedSensorPosition(0, 0, Constants.TALON_RUNNING_TIMEOUT_MS); //set the encoder position to the bottom
        }
    }

    /**
     * Get the velocity from the encoders
     *
     * @return velocity of the motor, in m/s
     */
    public double getSpeed() {
        return convertTicksToHeight(10 * masterMotor.getSelectedSensorVelocity(0)); //sensorVelocity calculates for 100ms
    }

    /**
     * Set the motor to a certain speed, on a scale of -1 to 1.
     *
     * @param speed speed of the motor from -1 to 1
     */
    public void setSpeed(double speed) {
        masterMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, Constants.FIRST_STAGE_FEEDFORWARD);
    }

    /**
     * Check if the limit switch at the top of the elevator is pressed
     *
     * @return boolean of the sensor true or false
     */
    public boolean atTop() {
        return masterMotor.getSensorCollection().isFwdLimitSwitchClosed() == !Constants.TOP_HALL_REVERSED;
    }

    /**
     * Check if the limit switch at the bottom of the elevator is pressed
     *
     * @return boolean of the sensor true or false
     */
    public boolean atBottom() {
        return masterMotor.getSensorCollection().isRevLimitSwitchClosed() == !Constants.BOTTOM_HALL_REVERSED;
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
     * checks if the elevator is in range of the genesis profile to make sure the hatch system isn't open when in that zone
     * @return
     */
    public boolean isSetpointInDangerZone() {
        return (getHeight() < Constants.UPPER_DANGER_ZONE && convertTicksToHeight(setpoint) > Constants.LOWER_DANGER_ZONE) ||
                (getHeight() > Constants.LOWER_DANGER_ZONE && convertTicksToHeight(setpoint) < Constants.UPPER_DANGER_ZONE);

    }


    /**
     * Get the speed of the motor in percentages
     *
     * @return return motor value, from -1 to 1
     */
    public double getOutputPrecent() {
        return masterMotor.getMotorOutputPercent();
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

    /**
     * resets the encoder of the elevator
     */
    public void resetEncoders() {
        masterMotor.setSelectedSensorPosition(Constants.START_UNIT, 0, Constants.TALON_RUNNING_TIMEOUT_MS);
        setHeight(0);
    }

    @Override
    public void initDefaultCommand() {
        //setDefaultCommand(new ElevatorCommand(1.2));
        setDefaultCommand(new JoystickElevatorCommand());
    }

    public void onDisabled() {
        setpoint = 0;
        masterMotor.set(ControlMode.PercentOutput, 0);
    }
}
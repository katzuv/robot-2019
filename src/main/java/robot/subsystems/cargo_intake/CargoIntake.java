/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.cargo_intake;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import robot.Robot;
import robot.subsystems.cargo_intake.commands.JoystickWristTurn;

import static robot.Robot.cargoIntake;

/**
 * The Cargo Intake subsystem, including the Intake and Wrist.
 * first the gripper
 *
 * @author lior
 */
public class CargoIntake extends Subsystem {
    private final TalonSRX wrist = new TalonSRX(Ports.WristMotor);
    private final AnalogInput proximitySensor = new AnalogInput(Ports.proximitySensor);
    private final VictorSPX IntakeMotor = new VictorSPX(Ports.IntakeMotor);
    private double setPointAngle;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public CargoIntake() {
        /*
        config for the feedback sensor
         */
        if (Constants.IS_MAG_ENCODER_RELATIVE)
            wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        else
            wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

        wrist.setSensorPhase(Constants.SENSOR_PHASE);
        wrist.setInverted(Constants.WRIST_MOTOR_REVERSED);
        wrist.overrideLimitSwitchesEnable(Constants.LIMIT_SWITCH_OVERRIDE);
        wrist.overrideSoftLimitsEnable(Constants.SOFT_LIMIT_OVERRIDE);



        /*
        PIDF config
         */
        wrist.config_kP(0, Constants.kP, Constants.TALON_TIME_OUT);
        wrist.config_kI(0, Constants.kI, Constants.TALON_TIME_OUT);
        wrist.config_kD(0, Constants.kD, Constants.TALON_TIME_OUT);
        wrist.config_kF(0, Constants.kF, Constants.TALON_TIME_OUT);
        wrist.config_IntegralZone(0, Constants.IZone, Constants.TALON_TIME_OUT);
        /*
        status frame period config
         */
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TALON_TIME_OUT);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TALON_TIME_OUT);
        /*
        nominal and peak output config
         */
        wrist.configNominalOutputForward(0., Constants.TALON_TIME_OUT);
        wrist.configNominalOutputReverse(0, Constants.TALON_TIME_OUT);
        wrist.configPeakOutputForward(Constants.PEAK_OUTPUT_FORWARD, Constants.TALON_TIME_OUT);
        wrist.configPeakOutputReverse(Constants.PEAK_OUTPUT_REVERSE, Constants.TALON_TIME_OUT); //TODO: change back to .5
        /*
        profile config
         */
        wrist.selectProfileSlot(0, 0);
        /*
        motion magic speed config
         */
        wrist.configMotionCruiseVelocity(Constants.CRUISE_VELOCITY, Constants.TALON_TIME_OUT);
        wrist.configMotionAcceleration(Constants.MOTION_MAGIC_ACCELERATION, Constants.TALON_TIME_OUT);
        /*
        limit switch config
         */
        wrist.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
         Constants.FORWARD_NORMALLY_CLOSED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
         Constants.TALON_TIME_OUT);
         wrist.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
         Constants.REVERSE_NORMALLY_CLOSED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
         Constants.TALON_TIME_OUT);


    }


    public double getProximityVoltage() {
        return proximitySensor.getVoltage();//proximitySensor.getVoltage();
    }//returns the current voltage in the proximity sensor

    public boolean isCargoInside() {
        return getProximityVoltage() > Constants.CARGO_IN_VOLTAGE;
    }

    public void setGripperSpeed(double speed) {
        IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setWristSpeed(double speed) {
        wrist.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, stallCurrent());
    }

    public void resetSensors() {
        resetProximitySensor();
        resetWristEncoder();
    }

    private void resetWristEncoder() {
        wrist.setSelectedSensorPosition(0, 0, Constants.TALON_TIME_OUT);
    }

    public double stallCurrent() {
        final double wristAngle = cargoIntake.getWristAngle();
        if (wristAngle < 6 && setPointAngle < 3) { //TODO: needs to be a constants
            return 0;
        }
        final double COMCosine = Math.cos(Math.toRadians(15 + cargoIntake.getWristAngle()));
        return 1.1 * (0.2 * COMCosine + 0.025 * Math.signum(COMCosine));
    }

    /**
     * Convert angle in degrees to ticks of the encoder.
     *
     * @param angle height in meters
     * @return ticks of the encoder
     */
    public int convertAngleToTicks(double angle) {
        return (int) (angle * Constants.TICKS_PER_DEGREE);
    }

    /**
     * Convert ticks of the encoder to angle in degrees.
     *
     * @param ticks ticks of the encoder
     * @return height in meters
     */
    public double convertTicksToAngle(int ticks) {
        return ticks / Constants.TICKS_PER_DEGREE;
    }

    /**
     * @return
     */
    public double getWristAngle() {
        return convertTicksToAngle(wrist.getSelectedSensorPosition());

    }
    /**
     * Beyond preventing the motors from going above a certain height, this method prevents them from moving higher or
     * lower once one of the limit switches/hall effects is pressed.
     */
    public void preventOverShoot() {
        if (atPeak()) {
            //setHeight(Math.min(getHeight(), convertTicksToMeters(setpoint)));
            //wrist.setSelectedSensorPosition((int) (Constants.WRIST_ANGLES.INTAKE.getValue() * Constants.TICKS_PER_DEGREE), 0, Constants.TALON_TIME_OUT); //set the position to the top.
        }
        if (atClosed()) {
            //setHeight(Math.max(getHeight(), convertTicksToMeters(setpoint)));
            //wrist.setSelectedSensorPosition(0, 0, Constants.TALON_TIME_OUT); //set the encoder position to the bottom
        }
    }

    /**
     * Check if the limit switch at the top of the elevator is pressed
     *
     * @return boolean of the sensor true or false
     */
    public boolean atPeak() {
        //return Math.abs(getHeight()) > Constants.ELEVATOR_MAX_HEIGHT;
        if (!Robot.isRobotA) {
            return wrist.getSensorCollection().isFwdLimitSwitchClosed() == !Constants.FORWARD_NORMALLY_CLOSED;
        }
        return false;
    }

    /**
     * Check if the limit switch at the bottom of the elevator is pressed
     *
     * @return boolean of the sensor true or false
     */
    public boolean atClosed() {
        //return Math.abs(getHeight()) < 0.05;
        if (!Robot.isRobotA) {
            return wrist.getSensorCollection().isRevLimitSwitchClosed() == !Constants.REVERSE_NORMALLY_CLOSED;
        }
        return false;
    }

    public void setWristAngle(double angle) {
        angle = Math.max(0,angle);
        angle = Math.min(Constants.WRIST_ANGLES.MAXIMAL.getValue(),angle);
        setPointAngle = angle;
        cargoIntake.wrist.set(ControlMode.MotionMagic, convertAngleToTicks(angle), DemandType.ArbitraryFeedForward, cargoIntake.stallCurrent());
    }

    public int getVelocity() {

        return wrist.getSelectedSensorVelocity();
    }

    @Override
    public void initDefaultCommand() {

        setDefaultCommand(new JoystickWristTurn());
    }

    private void resetProximitySensor() {
        //proximitySensor.resetAccumulator();
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.wrist_control;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.wrist_control.commands.JoystickWristTurn;

import static robot.Robot.wristControl;

/**
 * New wrist subsystem for the 2019 robot 'GENESIS', after the district championships.
 *
 * @author lior
 */
public class WristControl extends Subsystem {
    private final TalonSRX wrist = new TalonSRX(Ports.WristMotor);
    private double setPointAngle;
    private boolean raw = false;
    private double lastWristAngle = 0;

    public WristControl() {
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

        wrist.configContinuousCurrentLimit(10, Constants.TALON_TIME_OUT);
        wrist.configPeakCurrentLimit(30, Constants.TALON_TIME_OUT);
        wrist.configPeakCurrentDuration(300, Constants.TALON_TIME_OUT);
        wrist.enableCurrentLimit(true);

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
        wrist.configPeakOutputReverse(Constants.PEAK_OUTPUT_REVERSE, Constants.TALON_TIME_OUT);
        /*
        profile config
         */
        wrist.selectProfileSlot(0, 0);
        /*
        motion magic speed config
         */
        wrist.configMotionCruiseVelocity(Constants.CRUISE_VELOCITY, Constants.TALON_TIME_OUT);
        wrist.configMotionAcceleration(Constants.MOTION_MAGIC_ACCELERATION, Constants.TALON_TIME_OUT);

        wrist.configVoltageCompSaturation(12.0);
        wrist.enableVoltageCompensation(true);

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

    /**
     * Control the wrist manually
     *
     * @param speed
     */
    public void setWristSpeed(double speed) {
        if (speed != 0) //in cases where we set speed 0, we usually still want the wrist to hold itself in place.
            raw = true;
        wrist.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, stallCurrent());
    }

    /**
     * Reset the wrist encoders.
     */
    public void resetSensors() {
        resetWristEncoder();
    }

    private void resetWristEncoder() {
        lastWristAngle = 0;
        wrist.setSelectedSensorPosition(0);
    }

    /**
     * The wrist voltage which holds it in place
     *
     * @return
     */
    public double stallCurrent() {
        final double wristAngle = wristControl.getWristAngle(); //for readability
        if (dropWrist())
            return 0;
        double multiplier = Robot.gripperWheels.isCargoInside() ? Constants.CARGO_MULTIPLIER : 1; //TODO: add hatch comp aswell
        return multiplier * (
                (Constants.PEAK_PERCENT_COMPENSATION - Constants.ZERO_ANGLE_COMPENSATION) * Math.cos(Math.toRadians(Constants.COM_ANGLE + wristAngle))
                        + Constants.ZERO_ANGLE_COMPENSATION * Math.signum(Math.cos(Math.toRadians(Constants.COM_ANGLE + wristAngle))));

    }

    public void setEncoderAngle(double angle) {
        lastWristAngle = angle;
        wrist.setSelectedSensorPosition(convertAngleToTicks(angle));
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

    public void setWristAngle(double angle) {
        raw = false;
        angle = Math.max(0, angle);
        angle = Math.min(Constants.WRIST_ANGLES.MAXIMAL.getValue(), angle);
        if (Robot.debug) {
            SmartDashboard.putNumber("Cargo intake: target", angle);
        }
        setPointAngle = angle;
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

    public int getVelocity() {

        return wrist.getSelectedSensorVelocity();
    }

    @Override
    public void initDefaultCommand() {

        setDefaultCommand(new JoystickWristTurn());
    }

    /**
     * Detects if the difference in wrist angle from the last loop is higher than a constant. if so, the encoder jumped.
     *
     * @return returns true, if there was a detected jump.
     */
    public boolean preventEncoderJumps() {
        double currentAngle = getWristAngle();
        if (Math.abs(lastWristAngle - currentAngle) > Constants.WRIST_JUMP_ANGLE) {
            setEncoderAngle(lastWristAngle);
            return true;
        }

        lastWristAngle = currentAngle;
        return false;
    }

    /**
     * This method makes sure the wrist stallCurrent gets updated
     */
    public void update() {
        if (!raw) {
            if (dropWrist())
                wristControl.wrist.set(ControlMode.PercentOutput, 0);
            else
                wristControl.wrist.set(ControlMode.MotionMagic, convertAngleToTicks(setPointAngle), DemandType.ArbitraryFeedForward, wristControl.stallCurrent());
        }
    }

    public boolean dropWrist() {
        return (wristControl.getWristAngle() < Constants.DROP_WRIST_ANGLE &&
                setPointAngle < Constants.DROP_WRIST_ANGLE / 2) ||
                (Constants.WRIST_FORWARD_DROP_DISABLED || (wristControl.getWristAngle() > Constants.WRIST_ANGLES.MAXIMAL.getValue() - Constants.DROP_WRIST_ANGLE &&
                        setPointAngle > Constants.WRIST_ANGLES.MAXIMAL.getValue() - Constants.DROP_WRIST_ANGLE / 2));
    }

    @Override
    public void periodic() {
        update();
    }

    public void disabledPeriodic() {
        setPointAngle = getWristAngle();
    }

    /**
     * Update constants from the shuffleboard directly. useful for debugging
     */
    public void updateConstants() {
        Constants.WRIST_FORWARD_DROP_DISABLED = getConstant("Disable: wrist forward drop", Constants.WRIST_FORWARD_DROP_DISABLED);
        Constants.PROXIMITY_DISABLED = getConstant("Disable: Cargo proximity", Constants.PROXIMITY_DISABLED);
    }

    private double getConstant(String key, double constant) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, constant));
        return SmartDashboard.getNumber(key, constant);
    }

    private boolean getConstant(String key, boolean constant) {
        SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, constant));
        return SmartDashboard.getBoolean(key, constant);
    }
}

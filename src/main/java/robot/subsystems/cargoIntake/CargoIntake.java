/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.cargoIntake;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The Cargo Intake subsystem, including the Intake and Wrist.
 * first the gripper
 */
public class CargoIntake extends Subsystem {
//    private final AnalogInput proximitySensor = new AnalogInput(Ports.proximitySensor);
    private final VictorSPX IntakeMotor = new VictorSPX(Ports.IntakeMotor);
    private final TalonSRX wrist = new TalonSRX(Ports.WristMotor);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public CargoIntake() {
        /*
        config for the feedback sensor
         */
        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        wrist.setSensorPhase(Constants.SENSOR_PHASE);
        wrist.setInverted(Constants.WRIST_MOTOR_REVERSED);
        wrist.setSelectedSensorPosition(0, Constants.PID_LOOP_IDX, Constants.TALON_TIME_OUT);
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
        wrist.configNominalOutputForward(0, Constants.TALON_TIME_OUT);
        wrist.configNominalOutputReverse(0, Constants.TALON_TIME_OUT);
        wrist.configPeakOutputForward(0.5, Constants.TALON_TIME_OUT);
        wrist.configPeakOutputReverse(0.5, Constants.TALON_TIME_OUT);
        /*
        profile config
         */
        wrist.selectProfileSlot(Constants.SLOT_IDX, Constants.PID_LOOP_IDX);
        /*
        motion magic speed config
         */
        wrist.configMotionCruiseVelocity(15000, Constants.TALON_TIME_OUT);
        wrist.configMotionAcceleration(6000, Constants.TALON_TIME_OUT);
        /*
        limit switch config
         */
        wrist.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                Constants.WRIST_LIMIT_REVESED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIME_OUT);
        wrist.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                Constants.WRIST_LIMIT_REVESED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIME_OUT);


    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public double getProximityVoltage() {
        return 2;//proximitySensor.getVoltage();
    }//returns the current voltage in the proximity sensor

    public boolean isCargoInside() {
        return getProximityVoltage() > Constants.CARGO_IN_VOLTAGE;//felt cute might delete later
    }

    public void setGripperSpeed(double speed) {
        IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setWristPosition(double pos) {
        wrist.set(ControlMode.MotionMagic, pos);
    }

    public void resetWristEncoder() {
        wrist.setSelectedSensorPosition(0, 0, Constants.TALON_TIME_OUT);
    }

    /**
     * Convert angle in degrees to ticks of the encoder.
     *
     * @param angle height in meters
     * @return ticks of the encoder
     */
    private int convertAngleToTicks(double angle) {
        return (int) (angle * Constants.TICKS_PER_DEGREE);
    }

    /**
     * Convert ticks of the encoder to angle in degrees.
     *
     * @param ticks ticks of the encoder
     * @return height in meters
     */
    private double convertTicksToAngle(int ticks) {
        return ticks / Constants.TICKS_PER_DEGREE;
    }

    /**
     * @return
     */
    public double getWristAngle() {
        return convertTicksToAngle(wrist.getSelectedSensorPosition());
    }
}

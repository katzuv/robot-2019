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
    AnalogInput proximitySensor = new AnalogInput(Ports.proximitySensor);
    private final VictorSPX IntakeMotor = new VictorSPX(Ports.IntakeMotor);
    private final TalonSRX WristControlMotor = new TalonSRX(Ports.WristMotor);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public CargoIntake() {
        /*
        config for the feedback sensor
         */
        WristControlMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0,0);
        WristControlMotor.setSensorPhase(true);
        WristControlMotor.setInverted(false);
        WristControlMotor.setSelectedSensorPosition(0, Constants.PID_LOOP_IDX, Constants.TALON_TIME_OUT);
        /*
        PID config
         */
        WristControlMotor.config_kP(0, Constants.kP, Constants.TALON_TIME_OUT);
        WristControlMotor.config_kI(0, Constants.kI, Constants.TALON_TIME_OUT);
        WristControlMotor.config_kD(0, Constants.kD, Constants.TALON_TIME_OUT);
        WristControlMotor.config_kF(0, Constants.kF, Constants.TALON_TIME_OUT);
        /*
        status frame period config
         */
        WristControlMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TALON_TIME_OUT);
        WristControlMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TALON_TIME_OUT);
        /*
        nominal and peak output config
         */
        WristControlMotor.configNominalOutputForward(0, Constants.TALON_TIME_OUT);
        WristControlMotor.configNominalOutputReverse(0, Constants.TALON_TIME_OUT);
        WristControlMotor.configPeakOutputForward(1, Constants.TALON_TIME_OUT);
        WristControlMotor.configPeakOutputReverse(-1, Constants.TALON_TIME_OUT);
        /*
        profile config
         */
        WristControlMotor.selectProfileSlot(Constants.SLOT_IDX, Constants.PID_LOOP_IDX);
        /*
        motion magic speed config
         */
        WristControlMotor.configMotionCruiseVelocity(15000, Constants.TALON_TIME_OUT);
        WristControlMotor.configMotionAcceleration(6000, Constants.TALON_TIME_OUT);
        /*
        limit switch config
         */
        WristControlMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                Constants.WRIST_LIMIT_REVESED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIME_OUT);
        WristControlMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                Constants.WRIST_LIMIT_REVESED ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen,
                Constants.TALON_TIME_OUT);


    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public double proximityVoltage(){ return proximitySensor.getVoltage(); }//returns the current voltage in the proximity sensor

    public boolean isCargoInside(){
        return proximityVoltage() > Constants.CARGO_IN_VOLTAGE;//felt cute might delete later
    }

    public void setGripperSpeed(double speed) {
        IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setWristPosition(double pos) {
        WristControlMotor.set(ControlMode.MotionMagic, pos);
    }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.cargoIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The Cargo Intake subsystem, including the Intake and Wrist.
 * first the gripper
 */
public class CargoIntake extends Subsystem {
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
        WristControlMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.TimeOutMS);
        /*
        PID config
         */
        WristControlMotor.config_kP(0, Constants.kP, Constants.TimeOutMS);
        WristControlMotor.config_kI(0, Constants.kI, Constants.TimeOutMS);
        WristControlMotor.config_kD(0, Constants.kD, Constants.TimeOutMS);
        WristControlMotor.config_kF(0, Constants.kF, Constants.TimeOutMS);
        /*
        status frame period config
         */
        WristControlMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TimeOutMS);
        WristControlMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TimeOutMS);
        /*
        nominal and peak output config
         */
        WristControlMotor.configNominalOutputForward(0, Constants.TimeOutMS);
        WristControlMotor.configNominalOutputReverse(0, Constants.TimeOutMS);
        WristControlMotor.configPeakOutputForward(1, Constants.TimeOutMS);
        WristControlMotor.configPeakOutputReverse(-1, Constants.TimeOutMS);
        /*
        profile config
         */
        WristControlMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
        /*
        motion magic speed config
         */
        WristControlMotor.configMotionCruiseVelocity(15000, Constants.TimeOutMS);
        WristControlMotor.configMotionAcceleration(6000, Constants.TimeOutMS);


    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void setGripperSpeed(double speed) {
        IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setWristPosition(double pos) {
        WristControlMotor.set(ControlMode.MotionMagic, pos);
    }

}

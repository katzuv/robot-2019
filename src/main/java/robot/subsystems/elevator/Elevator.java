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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends Subsystem {

    private double setpoint;
    /* pid slots for the four states: up and down on the first level and up and down on the second level of the cascade*/
    private final int TALON_BOTTOM_UP_PID_SLOT = 0;
    private final int TALON_BOTTOM_DOWN_PID_SLOT = 1;
    private final int TALON_TOP_UP_PID_SLOT = 2;
    private final int TALON_TOP_DOWN_PID_SLOT = 3;

    private final VictorSPX victorMotor = new VictorSPX(Ports.victorMotor);
    private final TalonSRX talonMotor = new TalonSRX(Ports.talonMotor);
    private final Encoder encoder = new Encoder(Ports.encoderChannelA, Ports.encoderChannelB);

    public Elevator() {
        encoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

        //what the motor does when not given voltage (Brake - decelerate the motor, Coast - not stop the motor)
        victorMotor.setNeutralMode(NeutralMode.Coast);
        talonMotor.setNeutralMode(NeutralMode.Coast);

        /* set closed loop gains in slot0 */
        talonMotor.config_kP(TALON_BOTTOM_UP_PID_SLOT, Constants.LIFT_BOTTOM_UP_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kI(TALON_BOTTOM_UP_PID_SLOT, Constants.LIFT_BOTTOM_UP_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kD(TALON_BOTTOM_UP_PID_SLOT, Constants.LIFT_BOTTOM_UP_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kF(TALON_BOTTOM_UP_PID_SLOT, Constants.LIFT_BOTTOM_UP_PIDF[3], Constants.TALON_TIMEOUT_MS);
        /* set closed loop gains in slot1 */
        talonMotor.config_kP(TALON_BOTTOM_DOWN_PID_SLOT, Constants.LIFT_BOTTOM_DOWN_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kI(TALON_BOTTOM_DOWN_PID_SLOT, Constants.LIFT_BOTTOM_DOWN_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kD(TALON_BOTTOM_DOWN_PID_SLOT, Constants.LIFT_BOTTOM_DOWN_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kF(TALON_BOTTOM_DOWN_PID_SLOT, Constants.LIFT_BOTTOM_DOWN_PIDF[3], Constants.TALON_TIMEOUT_MS);
        /* set closed loop gains in slot2 */
        talonMotor.config_kP(TALON_TOP_UP_PID_SLOT, Constants.LIFT_TOP_UP_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kI(TALON_TOP_UP_PID_SLOT, Constants.LIFT_TOP_UP_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kD(TALON_TOP_UP_PID_SLOT, Constants.LIFT_TOP_UP_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kF(TALON_TOP_UP_PID_SLOT, Constants.LIFT_TOP_UP_PIDF[3], Constants.TALON_TIMEOUT_MS);
        /* set closed loop gains in slot3 */
        talonMotor.config_kP(TALON_TOP_DOWN_PID_SLOT, Constants.LIFT_TOP_DOWN_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kI(TALON_TOP_DOWN_PID_SLOT, Constants.LIFT_TOP_DOWN_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kD(TALON_TOP_DOWN_PID_SLOT, Constants.LIFT_TOP_DOWN_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonMotor.config_kF(TALON_TOP_DOWN_PID_SLOT, Constants.LIFT_TOP_DOWN_PIDF[3], Constants.TALON_TIMEOUT_MS);
        victorMotor.follow(talonMotor);

        talonMotor.setInverted(Constants.TALON_REVERSE);

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
    }

    /**
     *
     */
    public void setHeight(double height) {
        this.setpoint = Math.max(Constants.ELEVATOR_TOP_HEIGHT, Math.min(0 ,height)) * Constants.TICKS_PER_METER;
        updatePIDSlot();
        talonMotor.set(ControlMode.Position, setpoint);
    }

    /**
     * @return
     */
    public double getHeight() {
        return talonMotor.getSelectedSensorPosition(0) / Constants.TICKS_PER_METER;
    }

    /**
     * update pid slot, disable robot?
     */
    public void update() {
        updatePIDSlot();
    }

    /**
     *
     */
    private void updatePIDSlot() {
        if(setpoint > getHeight() * Constants.TICKS_PER_METER){
            if(Constants.ELEVATOR_MID_HEIGHT < getHeight() * Constants.TICKS_PER_METER)
                talonMotor.selectProfileSlot(TALON_TOP_UP_PID_SLOT, 0);
            else
                talonMotor.selectProfileSlot(TALON_BOTTOM_UP_PID_SLOT, 0);
        }
        else{
            if(Constants.ELEVATOR_MID_HEIGHT < getHeight() * Constants.TICKS_PER_METER)
                talonMotor.selectProfileSlot(TALON_TOP_DOWN_PID_SLOT, 0);
            else
                talonMotor.selectProfileSlot(TALON_BOTTOM_DOWN_PID_SLOT, 0);
        }
    }


    public void setSpeed(double speed) {
        talonMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getSpeed() {
        return talonMotor.getSelectedSensorVelocity(0) / Constants.TICKS_PER_METER;
    }

    public boolean atTop(){
        //return Math.abs(getHeight()) > Constants.ELEVATOR_MAX_HEIGHT;
        return talonMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean atBottom() {
        //return Math.abs(getHeight()) < 0.05;
        return talonMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public double getOutputPrecent() {
        return talonMotor.getMotorOutputPercent();
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
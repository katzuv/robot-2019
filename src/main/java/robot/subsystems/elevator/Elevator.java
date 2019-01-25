/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends Subsystem {

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
    }

    /**
     *
     */
    public void setHeight(double height) {

    }

    /**
     * @return
     */
    public double getHeight() {
        return 0;
    }

    /**
     * update pid slot, disable robot?
     */
    public void update() {

    }

    /**
     *
     */
    private void updatePIDSlot() {

    }

    public void setSpeed() {

    }

    public double getSpeed() {
        return 0;
    }

    public boolean atTop() {
        return false;
    }

    public boolean atBottom() {
        return false;
    }

    public double getOutputPrecent() {
        return 0;
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
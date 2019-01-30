/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Climb extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    TalonSRX talonUL = new TalonSRX(Ports.upLeftMotor);
    TalonSRX talonUR = new TalonSRX(Ports.upRightMotor);
    TalonSRX talonDL = new TalonSRX(Ports.downLeftMotor);
    TalonSRX talonDR = new TalonSRX(Ports.downRightMotor);

    public Climb(){
        talonUL.setInverted(Constants.UP_LEFT_TALON_REVERSE);
        talonUR.setInverted(Constants.UP_RIGHT_TALON_REVERSE);
        talonDL.setInverted(Constants.DOWN_LEFT_TALON_REVERSE);
        talonDR.setInverted(Constants.DOWN_RIGHT_TALON_REVERSE);

        //what the motor does when not given voltage (Brake - decelerate the motor, Coast - not stop the motor)
        talonUL.setNeutralMode(NeutralMode.Brake);
        talonUR.setNeutralMode(NeutralMode.Brake);
        talonDL.setNeutralMode(NeutralMode.Brake);
        talonDR.setNeutralMode(NeutralMode.Brake);

        /* set closed loop gains in slot0 */
        talonUL.config_kP(0, Constants.CLIMB_PIDF[0], Constants.TALON_TIMEOUT_MS);
        talonUL.config_kI(0, Constants.CLIMB_PIDF[1], Constants.TALON_TIMEOUT_MS);
        talonUL.config_kD(0, Constants.CLIMB_PIDF[2], Constants.TALON_TIMEOUT_MS);
        talonUL.config_kF(0, Constants.CLIMB_PIDF[3], Constants.TALON_TIMEOUT_MS);

    }
    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
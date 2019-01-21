/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.cargoIntake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class CargoIntake extends Subsystem {
    private final VictorSPX IntakeMotor = new VictorSPX(Ports.IntakeMotor);
    private final TalonSRX WristControlMotor = new TalonSRX(Ports.WristMotor);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public CargoIntake(){
        WristControlMotor.config_kP(0, Constants.kP, Constants.TimeOutMS);
        WristControlMotor.config_kP(0, Constants.kI, Constants.TimeOutMS);
        WristControlMotor.config_kP(0, Constants.kD, Constants.TimeOutMS);
        WristControlMotor.config_kP(0, Constants.kF, Constants.TimeOutMS);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void setIntakeSpeed(double speed){
        IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setWristPos(double pos){
        WristControlMotor.set(ControlMode.MotionMagic, pos);
    }

}
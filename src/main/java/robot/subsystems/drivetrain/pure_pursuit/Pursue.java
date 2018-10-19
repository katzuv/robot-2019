package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.command.Subsystem;

import robot.Robot;
import robot.subsystems.drivetrain.Drivetrain;


public class Pursue {
    
    private Point RobotCurrent = new Point(0,0);

    public void currentLocation()
    {
        RobotCurrent.setX(Math.cos(Robot.navx.getAngle())*(Robot.drivetrain.getLeftDistance()+Robot.drivetrain.getRightDistance())/2);
        RobotCurrent.setY(Math.sin(Robot.navx.getAngle())*(Robot.drivetrain.getLeftDistance()+Robot.drivetrain.getRightDistance())/2);
    }

}

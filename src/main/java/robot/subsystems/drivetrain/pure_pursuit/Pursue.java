package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.command.Subsystem;

import robot.Robot;
import robot.subsystems.drivetrain.Drivetrain;


public class Pursue {
    private double distance = (Robot.drivetrain.getLeftDistance()+Robot.drivetrain.getRightDistance())/2;
    private Point RobotCurrent = new Point(0,0);

    public void currentLocation()
    {
        RobotCurrent.setX(RobotCurrent.getX()+(Math.cos(Robot.navx.getAngle())*(Robot.drivetrain.getLeftDistance()+Robot.drivetrain.getRightDistance())/2));
        RobotCurrent.setY(RobotCurrent.getY()+(Math.sin(Robot.navx.getAngle())*(Robot.drivetrain.getLeftDistance()+Robot.drivetrain.getRightDistance())/2));
    }

}

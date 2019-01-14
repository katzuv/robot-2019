/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Point;

import static robot.Robot.drivetrain;

public class JoystickDrive extends Command {

    public JoystickDrive() {
        requires(drivetrain);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // 1: linear
        // 2: bell
        // 3: x^3
        // 4: e^C(x - 1)
        int option = 1;
        final int C = 5;
        double leftInput = -Robot.m_oi.leftStick.getY();
        double rightInput = -Robot.m_oi.rightStick.getY();

        double leftOutput, rightOutput;
        switch (option) {
            case 1:
                leftOutput = leftInput;
                rightOutput = rightInput;
                break;
            case 2:
                leftOutput = bell(leftInput, C);
                rightOutput = bell(rightInput, C);
                break;
            case 3:
                leftOutput = Math.pow(leftInput, 3);
                rightOutput = Math.pow(rightInput, 3);
                break;
            case 4:
                if (leftInput > 0) {
                    leftOutput = Math.pow(Math.E, C * (leftInput - 1));
                } else {
                    leftOutput = -Math.pow(Math.E, C * (-leftInput - 1));
                }
                if (rightInput > 0) {
                    rightOutput = Math.pow(Math.E, C * (rightInput - 1));
                } else {
                    rightOutput = -Math.pow(Math.E, C * (-rightInput - 1));
                }
                break;
            default:
                throw new IllegalArgumentException("Number must be 1-4");
        }
        drivetrain.setSpeed(leftOutput + fallControl(Robot.navx.getWorldLinearAccelY(),
                Constants.ROLL_AXIS),
                rightOutput + fallControl(Robot.navx.getWorldLinearAccelY(), Constants.ROLL_AXIS));
    }

    private double bell(double x, double c) {
        return 2 / (1 + Math.pow(Math.E, -c * x)) - 1;
    }

    /**
     * Calculate the change in speed for the robot
     *
     * @param currentAcceleration the current acceleration of the robot
     * @param rollAxis            the position of the roll axis
     * @author Orel
     */
    public double fallControl(double currentAcceleration, Point rollAxis) {
        double xDifference = rollAxis.getX() + Constants.CENTER_MASS_TO_AXIS_DISTANCE * Math.cos(Robot.navx.getAngle());
        double yDifference = rollAxis.getY() + Constants.CENTER_MASS_TO_AXIS_DISTANCE * Math.sin(Robot.navx.getAngle());
        double maxAcceleration = (xDifference * Constants.G) / yDifference;
        double accelerationTarget = maxAcceleration - Constants.ACCELERATION_MISTAKE;// take down the tolerance
        double target = (accelerationTarget - currentAcceleration) * Constants.Kjerk;
        double change = target - currentAcceleration;//the acceleration
        if (xDifference <= Constants.MIN_Xdiffrence) {
            change -= Constants.ACCELERATION_FIX;
        }
        return change;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}

package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;

import static robot.Robot.wristControl;

/**
 *
 */
public class RawWristTurn extends CommandBase {
    private double speed;
    private double timeout;
    private Timer timer = new Timer();
    public RawWristTurn(double speed, double timeout) {
        addRequirements(wristControl);
        this.speed = speed;
        this.timeout = timeout;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    public void initialize() {
        timer.reset();
        timer.start();
        wristControl.setWristSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        wristControl.setWristSpeed(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished()
    {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
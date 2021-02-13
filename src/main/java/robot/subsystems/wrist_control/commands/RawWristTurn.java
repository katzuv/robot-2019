package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.subsystems.wrist_control.WristControl;

/**
 *
 */
public class RawWristTurn extends CommandBase {
    private WristControl wristControl;
    private double speed;
    private double timeout;
    private Timer timer = new Timer();

    public RawWristTurn(WristControl wristControl, double speed, double timeout) {
        addRequirements(wristControl);
        this.wristControl = wristControl;
        this.speed = speed;
        this.timeout = timeout;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        wristControl.setWristSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        wristControl.setWristSpeed(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }
}
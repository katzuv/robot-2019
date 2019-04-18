package robot.subsystems.drivetrain.talon_profiling;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import edu.wpi.first.wpilibj.command.Command;

import static robot.Robot.drivetrain;

/**
 * Motion profile autonomous code, written for the Detroit championships.
 * Implements all methods in the drivetrain.
 */
public class TalonFollow extends Command {

    private final BufferedTrajectoryPointStream rightStream;
    private final BufferedTrajectoryPointStream leftStream;

    /**
     *
     * @param leftStream
     * @param rightStream
     */
    public TalonFollow(BufferedTrajectoryPointStream leftStream, BufferedTrajectoryPointStream rightStream) {
        this.leftStream = leftStream;
        this.rightStream = rightStream;
        requires(drivetrain);
    }

    /**
     *
     * @param leftStream
     * @param rightStream
     * @param isFlipped
     */
    public TalonFollow(BufferedTrajectoryPointStream leftStream, BufferedTrajectoryPointStream rightStream, boolean isFlipped) {
        requires(drivetrain);
        if(isFlipped)
        {
            this.leftStream = rightStream;
            this.rightStream = leftStream;
        }
        else
        {
            this.leftStream = leftStream;
            this.rightStream = rightStream;
        }
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        drivetrain.startMotionProfile(leftStream, rightStream);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return drivetrain.isMotionProfileDone();
    }

    // Called once after isFinished returns true
    protected void end() {
        drivetrain.setSpeed(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
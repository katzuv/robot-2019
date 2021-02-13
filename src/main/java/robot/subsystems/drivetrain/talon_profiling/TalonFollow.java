package robot.subsystems.drivetrain.talon_profiling;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.subsystems.drivetrain.Drivetrain;

/**
 * Motion profile autonomous code, written for the Detroit championships.
 * Implements all methods in the drivetrain.
 */
public class TalonFollow extends CommandBase {

    private final Drivetrain drivetrain;
    private final BufferedTrajectoryPointStream rightStream;
    private final BufferedTrajectoryPointStream leftStream;

    /**
     * @param leftStream
     * @param rightStream
     */
    public TalonFollow(Drivetrain drivetrain, BufferedTrajectoryPointStream leftStream, BufferedTrajectoryPointStream rightStream) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.leftStream = leftStream;
        this.rightStream = rightStream;
    }

    /**
     * @param leftStream
     * @param rightStream
     * @param isFlipped
     */
    public TalonFollow(Drivetrain drivetrain, BufferedTrajectoryPointStream leftStream, BufferedTrajectoryPointStream rightStream, boolean isFlipped) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        if (isFlipped) {
            this.leftStream = rightStream;
            this.rightStream = leftStream;
        } else {
            this.leftStream = leftStream;
            this.rightStream = rightStream;
        }
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        drivetrain.startMotionProfile(leftStream, rightStream);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return drivetrain.isMotionProfileDone();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }
}
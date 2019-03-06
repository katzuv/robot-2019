package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import static robot.Robot.drivetrain;

/**
 *
 */
public class DrivePathVision extends Command {

    private final TimedTrajectory<Pose2dWithCurvature> trajectory;

    public DrivePathVision(TimedTrajectory<Pose2dWithCurvature> trajectory) {
        this.trajectory = trajectory;
        requires(drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        drivetrain.trajectoryTracker.reset(trajectory);
        drivetrain.localization.reset(trajectory.getFirstState().getState().getPose());
        LiveDashboard.INSTANCE.setFollowingPath(true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double angle = Robot.visionTable.getEntry("tape_distance").getDouble(0);

        TrajectoryTrackerOutput trackerOutput = drivetrain.trajectoryTracker.nextState(drivetrain.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));

        LiveDashboard.INSTANCE.setPathX(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getX().getFeet());
        LiveDashboard.INSTANCE.setPathY(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getY().getFeet());
        LiveDashboard.INSTANCE.setPathHeading(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getRotation().getRadian());

        double linearVelocity = trackerOutput.getLinearVelocity().getValue(); // m/s
        double angularVelocity = trackerOutput.getAngularVelocity().getValue(); // rad/s

        if(angle != 0.0 && drivetrain.getRobotPosition().getTranslation().distance(trajectory.getLastState().getState().getPose().getTranslation()) < Constants.radiusFromEnd) {
            angularVelocity = angle * Constants.angleKp;
        }

        double tangentialVelocity = Constants.ROBOT_WIDTH / 2.0 * angularVelocity;

        double leftVelocity = linearVelocity - tangentialVelocity;
        double rightVelocity = linearVelocity + tangentialVelocity;
        drivetrain.setLeftVelocity(leftVelocity);
        drivetrain.setRightVelocity(rightVelocity);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return drivetrain.trajectoryTracker.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
        LiveDashboard.INSTANCE.setFollowingPath(false);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
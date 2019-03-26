package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import java.util.ArrayList;
import java.util.List;

import static robot.Robot.drivetrain;

/**
 *
 */
public class DrivePathVision extends Command {

    private TimedTrajectory<Pose2dWithCurvature> trajectory;
    private boolean stop = false;
    private final boolean reversed;
    private final List<Pose2d> waypoints;
    private final boolean vision;
    private final double startingVelocity;
    private final double endingVelocity;
    private final boolean includesStartingPoint;

    /**
     * Robot takes its current location and drives through waypoints in the list
     *
     * @param waypoints Target waypoints
     * @param reversed  Negative velocities
     */
    public DrivePathVision(List<Pose2d> waypoints, boolean reversed, boolean vision, double startingVelocity, double endingVelocity, boolean includesStartingPoint) {
        this.reversed = reversed;
        this.waypoints = waypoints;
        this.vision = vision;
        this.startingVelocity = startingVelocity;
        this.endingVelocity = endingVelocity;
        this.includesStartingPoint = includesStartingPoint;
        requires(drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        List<Pose2d> points = new ArrayList<>(waypoints);
        if (!includesStartingPoint) {
            points.add(0, drivetrain.getRobotPosition());
        }
        this.trajectory = TrajectoryGeneratorKt.getDefaultTrajectoryGenerator()
                .generateTrajectory(
                        points,
                        Constants.constraints,
                        VelocityKt.getVelocity(LengthKt.getMeter(startingVelocity)),
                        VelocityKt.getVelocity(LengthKt.getMeter(endingVelocity)),
                        VelocityKt.getVelocity(LengthKt.getMeter(2.7)),
                        AccelerationKt.getAcceleration(LengthKt.getMeter(2.7)),
                        reversed,
                        true
                );
        stop = false;
        drivetrain.trajectoryTracker.reset(trajectory);
        LiveDashboard.INSTANCE.setFollowingPath(true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double angle = Robot.visionTable.getEntry("tape_angle").getDouble(0);
        double distance = Robot.visionTable.getEntry("tape_distance").getDouble(0);

        TrajectoryTrackerOutput trackerOutput = drivetrain.trajectoryTracker.nextState(drivetrain.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));

        LiveDashboard.INSTANCE.setPathX(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getX().getFeet());
        LiveDashboard.INSTANCE.setPathY(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getY().getFeet());
        LiveDashboard.INSTANCE.setPathHeading(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getRotation().getRadian());

        double linearVelocity = trackerOutput.getLinearVelocity().getValue(); // m/s
        double angularVelocity = trackerOutput.getAngularVelocity().getValue(); // rad/s

        double distanceFromLast = drivetrain.getRobotPosition().getTranslation().distance(trajectory.getLastState().getState().getPose().getTranslation());

        SmartDashboard.putBoolean("using vision", angle != 0.0 && distanceFromLast < Constants.distanceFromEnd && vision);
        SmartDashboard.putNumber("Distance from last", distanceFromLast);
        if (distance != 0 && distance < 0.75 && vision) {
            stop = true;
        }

        if (angle != 0.0 && distanceFromLast < Constants.distanceFromEnd && vision) {
            angularVelocity = -Math.toRadians(angle) * Constants.pathAngleKp;
        }
        SmartDashboard.putBoolean("Stop", stop);
        if (stop) {
            angularVelocity = 0;
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
package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;
import robot.utilities.Utils;

import java.util.ArrayList;
import java.util.List;

import static robot.Robot.drivetrain;

/**
 *
 */
public class DrivePathVision extends Command {

    private final boolean vision;
    private double maxAcceleration;
    private double maxVelocity;
    private boolean reversed;
    private List<Pose2d> waypoints;
    private double startingVelocity;
    private double endingVelocity;
    private TimedTrajectory<Pose2dWithCurvature> trajectory;
    private boolean stop = false;

    /**
     * Robot takes its current location and drives through waypoints in the list
     *
     * @param trajectory Target waypoints
     * @param vision     Negative velocities
     */
    public DrivePathVision(TimedTrajectory<Pose2dWithCurvature> trajectory, boolean vision) {
        requires(drivetrain);
        this.trajectory = trajectory;
        this.vision = vision;
    }

    public DrivePathVision(List<Pose2d> points, double startingVelocity, double endingVelocity, boolean reversed, boolean vision) {
        this.vision = vision;
        this.reversed = reversed;
        this.waypoints = points;
        this.startingVelocity = startingVelocity;
        this.endingVelocity = endingVelocity;
        this.maxVelocity = Constants.RAMSETE_PEAK_VELOCITY;
        this.maxAcceleration = Constants.RAMSETE_PEAK_ACCELERATION;
    }


    public DrivePathVision(List<Pose2d> points, double startingVelocity, double endingVelocity, double maxVelocity, double maxAcceleration, boolean reversed, boolean vision) {
        this.vision = vision;
        this.reversed = reversed;
        this.waypoints = points;
        this.startingVelocity = startingVelocity;
        this.endingVelocity = endingVelocity;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (waypoints != null) {
            List<Pose2d> points = new ArrayList<>(waypoints);
            points.add(0, drivetrain.getRobotPosition());
            this.trajectory = Utils.generateTrajectory(
                    points, startingVelocity, endingVelocity, reversed
            );
        }
        drivetrain.trajectoryTracker.reset(trajectory);
        LiveDashboard.INSTANCE.setFollowingPath(true);
    }

    protected void execute() {
        //Get values from the smart dashboard
        double angle = Robot.visionTable.getEntry("tape_angle").getDouble(0);
        double distance = Robot.visionTable.getEntry("tape_distance").getDouble(0);

        TrajectoryTrackerOutput trackerOutput = drivetrain.trajectoryTracker.nextState(drivetrain.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));

        drivetrain.updateLiveDashboard();

        double linearVelocity = trackerOutput.getLinearVelocity().getValue(); // m/s
        double angularVelocity = trackerOutput.getAngularVelocity().getValue(); // rad/s

        double distanceFromLast = drivetrain.getRobotPosition().getTranslation().distance(trajectory.getLastState().getState().getPose().getTranslation());


        if (distance != 0 && distance < 0.75 && vision) {
            stop = true;
        }

        if (angle != 0.0 && distanceFromLast < Constants.distanceFromEnd && vision) {
            angularVelocity = -Math.toRadians(angle) * Constants.pathAngleKp;
        }

        //Debugging printing variables
        SmartDashboard.putBoolean("Debug: using vision", angle != 0.0 && distanceFromLast < Constants.distanceFromEnd && vision);
        SmartDashboard.putNumber("Debug: Distance from last", distanceFromLast);
        SmartDashboard.putBoolean("Debug: Stop", stop);

        if (stop) {
            angularVelocity = 0;
        }

        double tangentialVelocity = Constants.ROBOT_WIDTH / 2.0 * angularVelocity; //Multiply angular velocity by the robot radius to get the tangential velocity

//        drivetrain.setOutput(trackerOutput);

        drivetrain.setLeftVelocity(linearVelocity - tangentialVelocity);
        drivetrain.setRightVelocity(linearVelocity + tangentialVelocity);

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
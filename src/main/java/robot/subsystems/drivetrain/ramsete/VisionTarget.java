package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;

import java.util.ArrayList;
import java.util.List;

import static robot.Robot.drivetrain;

public class VisionTarget extends Command {
    private double last_angle = 0;
    private double last_distance = 0;
    private double last_field_angle = 0;
    private List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<>();
    private boolean reversed = true;

    public VisionTarget() {
        requires(drivetrain);
        constraints.add(new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getMeter(1.2192))));
        constraints.add(new VelocityLimitRegionConstraint(new Rectangle2d(LengthKt.getFeet(4), LengthKt.getFeet(7), LengthKt.getFeet(8), LengthKt.getFeet(20)), VelocityKt.getVelocity(LengthKt.getFeet(3))));
    }

    protected void initialize() {
        LiveDashboard.INSTANCE.setFollowingPath(true);
    }

    protected void execute() {
        double angle = Robot.visionTable.getEntry("last_angle").getDouble(0);
        double distance = Robot.visionTable.getEntry("last_distance").getDouble(0);
        double field_angle = Robot.visionTable.getEntry("last_field_angle").getDouble(0);

        if (distance > 0 && field_angle != last_field_angle && distance != last_distance && angle != last_angle) {
            drivetrain.trajectoryTracker.reset(generateTrajectory(angle, distance, field_angle));
            last_angle = angle;
            last_distance = distance;
            last_field_angle = field_angle;
        }

        TrajectoryTrackerOutput trackerOutput = drivetrain.trajectoryTracker.nextState(drivetrain.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));

        LiveDashboard.INSTANCE.setPathX(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getX().getFeet());
        LiveDashboard.INSTANCE.setPathY(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getY().getFeet());
        LiveDashboard.INSTANCE.setPathHeading(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getRotation().getRadian());

        double linearVelocity = trackerOutput.getLinearVelocity().getValue(); // m/s
        double angularVelocity = trackerOutput.getAngularVelocity().getValue(); // rad/s

        double tangentialVelocity = Constants.ROBOT_WIDTH / 2.0 * angularVelocity;

        double leftVelocity = linearVelocity - tangentialVelocity;
        double rightVelocity = linearVelocity + tangentialVelocity;
        drivetrain.setLeftVelocity(leftVelocity);
        drivetrain.setRightVelocity(rightVelocity);

    }

    private TimedTrajectory<Pose2dWithCurvature> generateTrajectory(double angle, double distance, double field_angle) {
        double angleModifier = 0;

        if (reversed) {
            angleModifier = 180;
        }

        angle = Math.toRadians(angle);

        List<Pose2d> list = new ArrayList<>();
        Pose2d robotPose = drivetrain.getRobotPosition();
        list.add(robotPose.transformBy(new Pose2d(Length.Companion.getKZero(), Length.Companion.getKZero(), Rotation2dKt.getDegree(angleModifier))));
        Pose2d calculatedPose = new Pose2d(LengthKt.getMeter(Math.abs(Math.cos(angle) * distance)), LengthKt.getMeter(-Math.abs(Math.sin(angle) * distance)), Rotation2dKt.getDegree(angleModifier + field_angle));
        list.add(robotPose.transformBy(calculatedPose));

        return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator()
                .generateTrajectory(
                        list,
                        constraints,
                        VelocityKt.getVelocity(LengthKt.getMeter((drivetrain.getLeftVelocity() + drivetrain.getRightVelocity()) / 2)),
                        VelocityKt.getVelocity(Length.Companion.getKZero()),
                        VelocityKt.getVelocity(LengthKt.getMeter(1.5)),
                        AccelerationKt.getAcceleration(LengthKt.getMeter(1.5)),
                        reversed,
                        true
                );
    }

    protected boolean isFinished() {
        return drivetrain.trajectoryTracker.isFinished();
    }

    protected void end() {
        LiveDashboard.INSTANCE.setFollowingPath(false);
    }

    protected void interrupted() {
        end();
    }
}
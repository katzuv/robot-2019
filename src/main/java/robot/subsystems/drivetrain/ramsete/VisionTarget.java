package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Point;
import robot.subsystems.drivetrain.pure_pursuit.Vector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static robot.Robot.drivetrain;

public class VisionTarget extends Command {
    private double last_angle = 0;
    private double last_distance = 0;
    private double last_field_angle = 0;
    private List<TimingConstraint<Pose2dWithCurvature>> constraints = new ArrayList<>();
    private boolean reversed = true;
    private boolean generate = true;
    private double distance;
    private double kP = 0.2;

    public VisionTarget(double distance) {
        this.distance = distance;
        requires(drivetrain);
        constraints.add(new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getMeter(1.2192))));
        constraints.add(new VelocityLimitRegionConstraint(new Rectangle2d(LengthKt.getFeet(4), LengthKt.getFeet(7), LengthKt.getFeet(8), LengthKt.getFeet(20)), VelocityKt.getVelocity(LengthKt.getFeet(3))));
    }

    protected void initialize() {
        LiveDashboard.INSTANCE.setFollowingPath(true);
        double angle = Robot.visionTable.getEntry("tape_angle").getDouble(0);
        double distance = Robot.visionTable.getEntry("tape_distance").getDouble(0);
        double field_angle = Robot.visionTable.getEntry("tape_field_angle").getDouble(0);
        last_distance = distance;
        drivetrain.trajectoryTracker.reset(generateTrajectory(angle, distance, field_angle));
        generate = true;
    }

    protected void execute() {
        double angle = Robot.visionTable.getEntry("tape_angle").getDouble(0);
        double distance = Robot.visionTable.getEntry("tape_distance").getDouble(0);
        double field_angle = Robot.visionTable.getEntry("tape_field_angle").getDouble(0);

//        if (distance > 0 && field_angle != last_field_angle && distance != last_distance && angle != last_angle && distance > 1) {
        System.out.println(last_distance + "|" + distance);
        if (distance != 0 && distance < 1.8 && generate) {
            drivetrain.trajectoryTracker.reset(generateTrajectory(angle, distance, field_angle));
            last_angle = angle;
            last_distance = distance;
            last_field_angle = field_angle;
            generate = false;
        }

        TrajectoryTrackerOutput trackerOutput = drivetrain.trajectoryTracker.nextState(drivetrain.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));

        LiveDashboard.INSTANCE.setPathX(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getX().getFeet());
        LiveDashboard.INSTANCE.setPathY(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getY().getFeet());
        LiveDashboard.INSTANCE.setPathHeading(drivetrain.trajectoryTracker.getReferencePoint().getState().getState().getPose().getRotation().getRadian());

        double linearVelocity = trackerOutput.getLinearVelocity().getValue(); // m/s
        double angularVelocity = trackerOutput.getAngularVelocity().getValue(); // rad/s

//        if(distance != 0 && distance > 1) {
//            angularVelocity = angle * kP;
//        }

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

        Vector direction = new Vector(distance, 0);

        angle = -angle;

        direction.rotate(drivetrain.getAngle() + angle);

        direction = direction.add(new Vector(-0.4, 0));

        SmartDashboard.putString("Vector", direction.toString());
        Point robotPoint = new Point(drivetrain.getRobotPosition().getTranslation().getX().getMeter(), drivetrain.getRobotPosition().getTranslation().getY().getMeter());
        Point calculatedPoint = direction.add(robotPoint);
        List<Pose2d> list = new ArrayList<>();
        Pose2d robotPose = drivetrain.getRobotPosition();
        Pose2d calculatedPose = new Pose2d(LengthKt.getMeter(calculatedPoint.getX()), LengthKt.getMeter(calculatedPoint.getY()), Rotation2dKt.getDegree(angleModifier));

        list.add(robotPose);
        list.add(calculatedPose);

        SmartDashboard.putString("calculatedPoint", calculatedPose.toString() + ": " + calculatedPose.getRotation().getDegree());

        return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator()
                .generateTrajectory(
                        list,
                        Collections.emptyList(),
                        VelocityKt.getVelocity(LengthKt.getMeter(0.5)),
                        VelocityKt.getVelocity(LengthKt.getMeter(0.5)),
                        VelocityKt.getVelocity(LengthKt.getMeter(1)),
                        AccelerationKt.getAcceleration(LengthKt.getMeter(1)),
                        reversed,
                        true
                );
    }
    //                        VelocityKt.getVelocity(LengthKt.getMeter((drivetrain.getLeftVelocity() + drivetrain.getRightVelocity()) / 2)),

    protected boolean isFinished() {
//        double distance = Robot.visionTable.getEntry("tape_distance").getDouble(0);
//        return distance != 0 && distance < 0.3;
        return drivetrain.trajectoryTracker.isFinished();
    }

    protected void end() {
        LiveDashboard.INSTANCE.setFollowingPath(false);
    }

    protected void interrupted() {
        end();
    }
}
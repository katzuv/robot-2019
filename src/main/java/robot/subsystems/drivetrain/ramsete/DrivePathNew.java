package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.Trajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;
import robot.Robot;
import robot.subsystems.drivetrain.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Point;

import java.util.ArrayList;
import java.util.List;

import static robot.Robot.drivetrain;

/**
 *
 */
public class DrivePathNew extends Command {
    private org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker ramseteTracker;
    private double currentTime = 0;
    private double lastLeftEncoder = 0;
    private double lastRightEncoder = 0;
    double initAngle = drivetrain.getAngle();
    private Point currentPoint = new Point(0, 0);


    public DrivePathNew() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        List<Pose2d> list = new ArrayList<>();
        list.add(new Pose2d(LengthKt.getMeter(0), LengthKt.getMeter(0), new Rotation2d(0)));
        list.add(new Pose2d(LengthKt.getMeter(2), LengthKt.getMeter(0), new Rotation2d(0)));
        list.add(new Pose2d(LengthKt.getMeter(2), LengthKt.getMeter(1), new Rotation2d(0)));
        Trajectory trajectory = TrajectoryGeneratorKt.getDefaultTrajectoryGenerator()
                .generateTrajectory(list, new ArrayList<>(), new Velocity(0, LengthKt.getMeter(0)), new Velocity(0, new Length(0)), new Velocity(0.8, new Length(0.8)), new Acceleration<>(0.5, new Length(0.5)), false, true);
        ramseteTracker = new org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker(2, 0.7);
        ramseteTracker.reset(trajectory);
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        currentTime = 0;
        initAngle = drivetrain.getAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        robot.subsystems.drivetrain.pure_pursuit.Point robot = drivetrain.currentLocation;
        TrajectoryTrackerOutput output = ramseteTracker.nextState(new Pose2d(LengthKt.getMeter(robot.getY()), LengthKt.getMeter(robot.getX()), new Rotation2d(drivetrain.getAngle())), TimeUnitsKt.getSecond(currentTime));
        double linearVelocity = output.getLinearVelocity().getValue();
        double angularVelocity = -1 * output.getAngularVelocity().getValue();
        double leftVelocity = linearVelocity - (angularVelocity * Constants.ROBOT_WIDTH / 2);
        double rightVelocity = linearVelocity + (angularVelocity * Constants.ROBOT_WIDTH / 2);
        System.out.println("Left: " + leftVelocity + " | Right: " + rightVelocity);
        Robot.drivetrain.setLeftFeedForward(leftVelocity);
        Robot.drivetrain.setRightFeedForward(rightVelocity);
        currentTime += 0.02;
        updatePoint();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        Point lastPoint = new Point(0, 1);
        boolean closeToLast = (drivetrain.currentLocation.getX() >= lastPoint.getX() - 0.1 &&
                drivetrain.currentLocation.getY() >= lastPoint.getY() - 0.1);
        return (closeToLast &&
                drivetrain.getLeftSpeed() < robot.subsystems.drivetrain.pure_pursuit.Constants.STOP_SPEED_THRESH &&
                drivetrain.getRightSpeed() < robot.subsystems.drivetrain.pure_pursuit.Constants.STOP_SPEED_THRESH);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    private void updatePoint() {
        //change in (change left encoder value + change in right encoder value)/2
        double distance = ((drivetrain.getLeftDistance() - lastLeftEncoder) + (drivetrain.getRightDistance() - lastRightEncoder)) / 2;
        int direction = 1;
        //update the x, y coordinates based on the robot angle and the distance the robot moved.
        currentPoint.setX(currentPoint.getX() + direction * distance * Math.sin((drivetrain.getAngle() - initAngle) * (Math.PI / 180.0)));
        currentPoint.setY(currentPoint.getY() + direction * distance * Math.cos((drivetrain.getAngle() - initAngle) * (Math.PI / 180.0)));

        //updates values for next run
        lastLeftEncoder = drivetrain.getLeftDistance();
        lastRightEncoder = drivetrain.getRightDistance();
        drivetrain.currentLocation.setX(currentPoint.getX());
        drivetrain.currentLocation.setY(currentPoint.getY());
        SmartDashboard.putNumber("CurrentLocX", currentPoint.getX());
        SmartDashboard.putNumber("CurrentLocY", currentPoint.getY());
    }
}

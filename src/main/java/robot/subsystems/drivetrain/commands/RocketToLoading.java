package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.*;

/**
 *
 */
public class RocketToLoading extends Command {

    NetworkTableEntry distanceEntry;
    NetworkTableEntry angleEntry;

    public RocketToLoading() {

    }
        // Called just before this Command runs the first time
        protected void initialize() {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            NetworkTable table = inst.getTable("vision");
            angleEntry = table.getEntry("angle");
            distanceEntry = table.getEntry("distance");

            Path path = new Path();
            path.appendWaypoint(new Waypoint(0,-3.5));
            path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
            PurePursue pursue = new PurePursue(path, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
            pursue.start();
        }

        // Called repeatedly when this Command is scheduled to run
        protected void execute() {
                }


        // Make this return true when this Command no longer needs to run execute()
        protected boolean isFinished() {
            return Point.distance(Robot.drivetrain.currentLocation, target(angleEntry.getDouble(0), distanceEntry.getDouble(0)))
                    <= 0;
        }

        // Called once after isFinished returns true
        protected void end() {
        }

        // Called when another command which requires one or more of the same
        // subsystems is scheduled to run
        protected void interrupted() {
        }

        private Path generateFromVision(double angle, double distance) {
            double targetDistance = distance / 100;
            Waypoint target = target(angle, targetDistance);
            Waypoint middleWP = getMiddleWP(target);
            Path path1 = new Path(new Waypoint[]{new Waypoint(0, 0), middleWP, target});
            System.out.println(path1);
            return path1;
        }

        private Waypoint getMiddleWP(Waypoint target) {
            return new Waypoint(0, target.getY() - target.getY() / 2);
        }

        private Waypoint target(double angle, double distance) {
            return new Waypoint(Math.sin(Math.toRadians(angle)) * distance + 0.15, Math.cos(Math.toRadians(angle)) * distance);
        }
}
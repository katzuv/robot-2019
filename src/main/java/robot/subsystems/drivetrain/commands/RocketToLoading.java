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
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

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
}
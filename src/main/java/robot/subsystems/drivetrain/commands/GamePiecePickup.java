package robot.subsystems.drivetrain.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.PurePursue;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

import static robot.Robot.*;

/**
 *
 */
public class GamePiecePickup extends Command {

    public GamePiecePickup() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    private NetworkTableEntry targetAngleEntry;
    private NetworkTableEntry targetDistanceEntry;

    // Called just before this Command runs the first time
    protected void initialize() {
        if (cargoIntake.isCargoInside())
            visionTable.getEntry("game_piece").setString("cargo");
        else
            visionTable.getEntry("game_piece").setString("hatch");
        targetAngleEntry = Robot.visionTable.getEntry("cargo_angle");
        targetDistanceEntry = Robot.visionTable.getEntry("cargo_distance");

        double targetDistance = targetDistanceEntry.getDouble(0)/100;
        double targetAngle = targetAngleEntry.getDouble(0);
        Waypoint target = new Waypoint(Math.sin(Math.toRadians(targetAngle)) * targetDistance +0.15, Math.cos(Math.toRadians(targetAngle)) * targetDistance   );
        Waypoint middleWP = new Waypoint(0, target.getY()-target.getY()/2);
        Path path1 = new Path(new Waypoint[]{new Waypoint(0,0), middleWP ,  target});
        System.out.println(path1);
        path1.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        SmartDashboard.putNumber("target distance", targetDistance);
        SmartDashboard.putNumber("target angle ", targetAngle);
        PurePursue pursue = new PurePursue(path1, Constants.LOOKAHEAD_DISTANCE, Constants.kP, Constants.kA, Constants.kV, true, false);
        pursue.start();
        System.out.println(path1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
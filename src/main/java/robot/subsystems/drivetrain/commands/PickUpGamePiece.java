package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.pure_pursuit.Constants;
import robot.subsystems.drivetrain.pure_pursuit.Path;
import robot.subsystems.drivetrain.pure_pursuit.Waypoint;

import static robot.Robot.navx;

/**
 *
 */
public class PickUpGamePiece extends CommandGroup {

    public PickUpGamePiece() {
        double targetDistance = 0;//will later be changed to the dashboard input
        double targetAngle = 0;//will later be changed to the dashboard input
        Waypoint target = new Waypoint(Math.sin(targetAngle) * targetDistance, Math.cos(targetAngle) * targetDistance);
        Waypoint middleWP = new Waypoint(Math.tan(navx.getAngle()) * target.getY(), target.getY());
        Path path = new Path(new Waypoint[]{middleWP, target});
        path.generateAll(Constants.WEIGHT_DATA, Constants.WEIGHT_SMOOTH, Constants.TOLERANCE, Constants.MAX_ACCEL, Constants.MAX_PATH_VELOCITY);
        //addSequential(purepursue) need new version of code
        addSequential(new HatchOrCargo(new HatchIntake(), new CargoIntake()));

        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
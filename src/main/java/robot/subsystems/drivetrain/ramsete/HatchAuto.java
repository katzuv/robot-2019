package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import robot.subsystems.elevator.Constants;
import robot.subsystems.hatch_intake.commands.PlaceHatch;

/**
 *
 */
public class HatchAuto extends CommandGroup {

    public HatchAuto(TimedTrajectory<Pose2dWithCurvature> trajectory) {
        addSequential(new DrivePathNew(trajectory));
        addSequential(new PlaceHatch(Constants.ELEVATOR_STATES.LEVEL1_HATCH));
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
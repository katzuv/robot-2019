package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import robot.subsystems.elevator.Constants;
import robot.subsystems.hatch_intake.commands.PlaceHatch;

/**
 *
 */
public class DriveWithVision extends CommandGroup {

    public DriveWithVision(TimedTrajectory<Pose2dWithCurvature> trajectory) {
        addSequential(new DrivePathNew(trajectory));
        addSequential(new VisionTarget());
        addSequential(new PlaceHatch(Constants.ELEVATOR_STATES.SHIP_CARGO));
    }
}
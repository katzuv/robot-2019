package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import robot.subsystems.hatch_intake.commands.TakeHatch;

/**
 *
 */
public class TrajectoryStation extends CommandGroup {

    public TrajectoryStation(TimedTrajectory<Pose2dWithCurvature> trajectory) {
//        addSequential(new DrivePathNew(trajectory));
//        addSequential(new VisionTarget(0.6));
        addSequential(new DrivePathVision(trajectory));
        addSequential(new TakeHatch());
    }
}

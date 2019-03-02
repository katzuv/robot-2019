package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

/**
 *
 */
public class DriveWithVision extends CommandGroup {

    public DriveWithVision(TimedTrajectory<Pose2dWithCurvature> trajectory) {
        addSequential(new DrivePathNew(trajectory));
        addSequential(new VisionTarget());
    }
}
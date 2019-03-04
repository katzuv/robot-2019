package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import robot.subsystems.elevator.Constants;
import robot.subsystems.hatch_intake.commands.PlaceHatch;
import robot.subsystems.hatch_intake.commands.TakeHatch;

/**
 *
 */
public class LoadingStation extends CommandGroup {

    public LoadingStation() {
//        addSequential(new DrivePathNew(trajectory));
        addSequential(new VisionTarget());
        addSequential(new TakeHatch());
    }
}
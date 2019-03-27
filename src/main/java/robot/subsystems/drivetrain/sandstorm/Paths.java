package robot.subsystems.drivetrain.sandstorm;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.utilities.Utils;

import java.util.ArrayList;
import java.util.Arrays;

public class Paths {
    public static final TimedTrajectory<Pose2dWithCurvature> RIGHT_HAB_TO_NEAR_ROCKET;
    static{
        RIGHT_HAB_TO_NEAR_ROCKET = Utils.generateTrajectory(
                Arrays.asList(
                        new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(9.408), Rotation2dKt.getDegree(180)),
                        new Pose2d(LengthKt.getFeet(11.68), LengthKt.getFeet(4.789), Rotation2dKt.getDegree(150))
                ), 0, 0, true
        );

    }
}

package robot.utilities;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import robot.subsystems.drivetrain.Constants;

import java.util.List;

public class Utils {

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double constrainedMap(double x, double in_min, double in_max, double out_min, double out_max) {
        return Math.max(out_min, Math.min(out_max, map(x, in_min, in_max, out_min, out_max)));
    }

    public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> points, double startingVelocity, double endingVelocity, boolean reversed) {
       return generateTrajectory(points, startingVelocity, endingVelocity, Constants.RAMSETE_PEAK_VELOCITY, Constants.RAMSETE_PEAK_ACCELERATION, reversed);
    }


    public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> points, double startingVelocity, double endingVelocity, double maxVelocity, double maxAcceleration, boolean reversed) {
        return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator()
                .generateTrajectory(
                        points,
                        Constants.constraints,
                        VelocityKt.getVelocity(LengthKt.getMeter(startingVelocity)),
                        VelocityKt.getVelocity(LengthKt.getMeter(endingVelocity)),
                        VelocityKt.getVelocity(LengthKt.getMeter(maxVelocity)),
                        AccelerationKt.getAcceleration(LengthKt.getMeter(maxAcceleration)),
                        reversed,
                        true
                );
    }
}

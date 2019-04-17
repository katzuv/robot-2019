package robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import robot.subsystems.drivetrain.Constants;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
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


    /**
     * Reads and processes individual data points from a CSV file.
     *
     * @param path the path of the file with the data points
     * @return an ArrayList of double[] with each data point
     */
    public static ArrayList<double[]> readCSVMotionProfileFile(String path) {

        ArrayList<double[]> pathSegments = new ArrayList<>();

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {

            String line;
            String csvDelimiter = ", ";

            while ((line = br.readLine()) != null) {
                String[] segment = line.split(csvDelimiter);
                try {
                    double[] convertedSegment = Arrays.stream(segment)
                            .mapToDouble(Double::parseDouble)
                            .toArray();
                    pathSegments.add(convertedSegment);
                } catch (NumberFormatException ignored) {
                }
            }

        } catch (IOException ex) {
            DriverStation.reportError("Unable to read motion profile file " + path, true);
        }

        return pathSegments;

    }
}

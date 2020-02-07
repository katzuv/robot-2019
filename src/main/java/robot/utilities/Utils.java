package robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

public class Utils {

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double constrainedMap(double x, double in_min, double in_max, double out_min, double out_max) {
        return Math.max(out_min, Math.min(out_max, map(x, in_min, in_max, out_min, out_max)));
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
            String csvDelimiter = ",";

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

    public static double velocityByDistance(double targetSpeed, double acceleration, double startPos, double targetPos) {
        return Math.sqrt(targetSpeed * targetSpeed + 2 * Math.abs(acceleration) * Math.abs(targetPos - startPos));
    }

}

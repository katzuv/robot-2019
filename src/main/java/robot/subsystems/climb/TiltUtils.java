package robot.subsystems.climb;

import javafx.geometry.Point3D;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class TiltUtils {
    public Point3D getRotatedPoint(Point3D p, double pitch, double roll, double distanceRight, double distanceForward){
        pitch = Math.toRadians(pitch);
        roll = Math.toRadians(roll);

        //Here we use euclidian rotation to rotate the point to its current position
        return new Point3D(distanceForward * cos(pitch) + distanceRight * sin(pitch) * sin(roll),
                distanceRight * cos(roll),
                distanceForward * sin(pitch) + distanceRight * cos(pitch));
    }
}

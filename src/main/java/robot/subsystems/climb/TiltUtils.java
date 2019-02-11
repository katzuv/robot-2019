package robot.subsystems.climb;

import javafx.geometry.Point2D;
import javafx.geometry.Point3D;

import static java.lang.Math.*;

public class TiltUtils {
    private static Point3D getRotatedPoint(double distanceRight, double distanceForward, double pitch, double roll) {
        pitch = Math.toRadians(pitch);
        roll = Math.toRadians(roll);

        /*
        The calculations here are done using Rotation matrices.
        the final dot is rotated around the y axis (pitch angle) and then around the x axis (roll angle).
        the multiplication of both rotation matrices looks something like this:

        |      cos phi        ,     0     , ... | | w |
        | sin theta * sin phi , cos theta , ... | | h |
        | sin phi * cos theta , sin theta , ... | | 0 |

        theta - pitch
        phi - roll
        w - l/r distance of the point
        h - f/b distance of the point

        for a full explanation about rotational matrices see: https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
         */
        return new Point3D(distanceRight * cos(roll),
                distanceRight * sin(pitch) * sin(roll) + distanceForward * cos(pitch),
                -distanceRight * sin(roll) * cos(pitch) + distanceForward * sin(pitch)); //todo: im 25% percent sure i fricked something up when moving from right hand math to left hand programming.

    }//TODO: assert length similarity for testing

    private static Point3D getRotatedPoint(Point2D point, double pitch, double roll) {
        return getRotatedPoint(point.getX(), point.getY(), pitch, roll);
    }

    private static double getLegDisplacement(Point3D displacedArm) {
        /*
        in this method, we use basic trigonometry to get the length of the arm, using its height off the ground
         */
        double h = getLegHeightOffGround(displacedArm);
        return h / sqrt(1 - pow(h / displacedArm.magnitude(), 2));
    }

    private static double getLegHeightOffGround(Point3D displacedArm) {
        return displacedArm.getZ();
    }

    public static double getLegLength(double legXDimension, double legYDimension, double pitch, double roll) {
        return getLegDisplacement(getRotatedPoint(new Point2D(legXDimension, legYDimension), pitch, roll));
    }
}

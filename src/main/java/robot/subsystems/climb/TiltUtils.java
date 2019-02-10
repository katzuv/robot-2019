package robot.subsystems.climb;

import javafx.geometry.Point2D;
import javafx.geometry.Point3D;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.pow;

public class TiltUtils {
    private static Point3D getRotatedPoint( double distanceRight, double distanceForward, double pitch, double roll){
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
                - distanceRight * sin(roll) * cos(pitch) + distanceForward * sin(pitch)); //todo: im 25% percent sure i fricked something up when moving from right hand math to left hand programming.

    }//TODO: assert length similarity for testing

    private static Point3D getRotatedPoint(Point2D point, double pitch, double roll){
        return getRotatedPoint(point.getX(), point.getY(), pitch, roll);
    }

    private static double getLegDisplacement(Point3D displacedArm, Point2D armDimension){
        /*
        in this method we get the current robot leg and where it should be,
        then we calculate the angle between them and the center of the robot,
        and then we use that angle and trigonometry to get the distance the leg would need to shorten up,
        so that it would make the robot be flat.
         */
        Point3D floor = new Point3D(armDimension.getX(), armDimension.getY(), 0); //assert that a 3d point wont be inputted by accident
        double cosangle = displacedArm.dotProduct(floor) / (displacedArm.magnitude() * floor.magnitude());
        return sqrt(1 - pow(cosangle,2)) * displacedArm.magnitude() / cosangle;
    }

    public static double getLegLength(Point2D armDimension, double pitch, double roll){
        return getLegDisplacement(getRotatedPoint(armDimension, pitch, roll), armDimension);
    }
}

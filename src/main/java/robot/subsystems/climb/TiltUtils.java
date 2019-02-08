package robot.subsystems.climb;

import javafx.geometry.Point2D;
import javafx.geometry.Point3D;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.pow;

public class TiltUtils {
    public Point3D getRotatedPoint( double distanceRight, double distanceForward, double pitch, double roll){
        pitch = Math.toRadians(pitch);
        roll = Math.toRadians(roll);

        //Here we use euclidian rotation to rotate the point to its current position
        return new Point3D(distanceForward * cos(pitch) + distanceRight * sin(pitch) * sin(roll),
                distanceRight * cos(roll),
                distanceForward * sin(pitch) + distanceRight * cos(pitch)); //todo: im 90% percent sure i fricked something up when moving from right hand math to left hand programming.

    }//TODO: assert length similarity

    public Point3D getRotatedPoint(Point2D point, double pitch, double roll){
        return getRotatedPoint(point.getX(), point.getY(), pitch, roll);
    }

    public double getlegLength(Point3D displacedArm, Point2D armDimension){
        Point3D floor = new Point3D(armDimension.getX(), armDimension.getY(), 0); //assert that a 3d point wont be inputted by accident
        double cosangle = displacedArm.dotProduct(floor) / (displacedArm.magnitude() * floor.magnitude());
        return sqrt(1 - pow(cosangle,2)) * pow(displacedArm.magnitude(),2) * floor.magnitude() / displacedArm.dotProduct(floor);
    }
}

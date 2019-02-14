package robot.subsystems.climb;

import robot.subsystems.drivetrain.pure_pursuit.Point;
import robot.subsystems.drivetrain.pure_pursuit.Point3D;

import static java.lang.Math.*;

/**
 * A class used for the mathematical calculations the robot needs to do when climbing.
 *
 * @author paulo
 */
public class TiltUtils {
    /**
     * Rotates a point around the origin(0,0) around the x(pitch) and y(roll) axes.
     * Note that this method is used purely for the climbing mechanism, as such there are several restraints.
     * the original point has to be on the XY plane, and the point can't be rotated around the Z axis.
     * this restraint was placed to prevent any mistakes or misunderstanding when working on the code.
     *
     * @param distanceRight   the X coordinate of the point.
     * @param distanceForward the Y coordinate of the point.
     * @param pitch           angle in degrees to rotate around the x axis.
     * @param roll            angle in degrees to rotate around the y axis.
     * @return the rotated point, on the 3D plane.
     */
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
                -distanceRight * sin(roll) * cos(pitch) + distanceForward * sin(pitch));

    }

    /**
     * Rotates a point around the origin(0,0) around the x(pitch) and y(roll) axes.
     * Note that this method is used purely for the climbing mechanism, as such there are several restraints.
     * the original point has to be on the XY plane, and the point can't be rotated around the Z axis.
     * this restraint was placed to prevent any mistakes or misunderstanding when working on the code.
     *
     * @param point the original point to be rotated.
     * @param pitch angle in degrees to rotate around the x axis.
     * @param roll  angle in degrees to rotate around the y axis.
     * @return the rotated point, on the 3D plane.
     */
    private static Point3D getRotatedPoint(Point point, double pitch, double roll) {
        return getRotatedPoint(point.getX(), point.getY(), pitch, roll);
    }

    /**
     * Uses trigonometry to get the legs error from where it should be.
     *
     * @param displacedArm The dimensions of the leg, in relation to its ideal state (where the robot is not tilted, or in other words if the point was on the XY plane)
     * @return the length in meters of the robot leg error.
     */
    private static double getLegDisplacement(Point3D displacedArm) {
        /*
        in this method, we use basic Pythagoras to get the length of the arm, using its height off the ground
             L
            /|`.
           / |  `.
          /  |    `.
         /   | h    `.
        /    |        `.
       C ----D---------- B
       C is the robots center point, CL being the line between the center point and the leg motor.
       B is the error the leg has from where it should be. note that CLB create a right angle.
       LB / LD = CL / CD  |  CD^2 + LD^2 = CL^2  |   LB = CL * LD / sqrt(CL^2 - LD^2)   |   LB = LD / sqrt(1 - (LD/CL)^2) (by taking out the common CL.
         */
        double h = getLegHeightOffGround(displacedArm);
        return h / sqrt(1 - pow(h / displacedArm.magnitude(), 2));
    }

    /**
     * Get the height of a point above the XY plane.
     *
     * @param displacedArm an instance of the Point3D class.
     * @return its height above the ground in meters
     */
    private static double getLegHeightOffGround(Point3D displacedArm) {
        return displacedArm.getZ();
    }

    /**
     * Calculates the length in meters each leg is extended more than it should be.
     *
     * @param legXDimension the mechanical distance of the leg on the robot chassis, perpendicular to the robots line of view.
     * @param legYDimension the mechanical distance of the leg on the robot chassis, parallel to the robots line of view.
     * @param pitch         the angle in degrees the robot is rotated along its Y axis.
     * @param roll          the angle in degrees the robot is rotated along its X axis.
     * @return the error of the leg in meters.
     */
    public static double getLegLength(double legXDimension, double legYDimension, double pitch, double roll) {
        return getLegDisplacement(getRotatedPoint(new Point(legXDimension, legYDimension), pitch, roll));
    }
}

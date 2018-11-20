package robot.subsystems.drivetrain.pure_pursuit;

import robot.subsystems.drivetrain.pure_pursuit.Constants;

import java.util.ArrayList;
import java.util.Arrays;


/**
 * @author Paulo Khayat
 * @author Lior Barkai
 * This class is the instance of the path, holding the points.
 */
public class Path {
    private ArrayList<Waypoint> path = new ArrayList<>();

    /**
     * Create an empty Path instance
     */
    public Path() {

    }

    /**
     * Create a Path instance
     *
     * @param array the array of points to add into the arraylist
     */
    public Path(Waypoint[] array) {
        path.addAll(Arrays.asList(array));
    }

    public Path(ArrayList<Waypoint> w) {
        path.addAll(w);
    }

    public static double[][] doubleArrayCopy(double[][] arr) {
        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];
        for (int i = 0; i < arr.length; i++) {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];
            //Copy Contents
            for (int j = 0; j < arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }
        return temp;
    }

    /**
     * Set a point at an index.
     *
     * @param index index of the desired point starting at zero, use -1 for last Point.
     */
    public void setWaypoint(int index, Waypoint p) {
        if (!(index <= path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException();
        if (index == path.size())
            this.appendWaypoint(p);
        else
            path.set(index % path.size(), p);
    }

    /**
     * Adds point to the path.
     *
     * @param index the index of the point to append.
     * @param p     the waypoint to add.
     */
    public void addWaypoint(int index, Waypoint p) {
        if (!(index <= path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException();
        if (index == path.size())
            this.appendWaypoint(p);
        else
            path.add(index % path.size(), p);
    }

    /**
     * Appends an object to the end of the list.
     *
     * @param p the waypoint to add.
     */
    public void appendWaypoint(Waypoint p) {
        path.add(p);
    }

    /**
     * Gives a new subPath of the path.
     *
     * @param start  first index of the path
     * @param length last index (subtracted by one)
     * @return returns a new Path class which holds the specified sublist
     */
    public Path getSubPath(int start, int length) {
        return new Path(new ArrayList<>(path.subList(start, length)));
    }

    /**
     * get a Point at a specific index.
     *
     * @param index index of the desired point starting at zero, use -1 for last Point.
     * @return returns the Point.
     */
    public Waypoint getWaypoint(int index) {
        if (!(index < path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException();
        if (path.get(index % path.size()) == null)
            throw new ClassCastException("Tried to call a non Point object from the path list.");
        return path.get(index % path.size());
    }

    /**
     * Adds all of a Point array to the end of the path list.
     * The equivalent of 'addAll(-1, array)'
     *
     * @param array the array of points to add to the end of the array.
     */
    public void addAll(Waypoint[] array) {
        path.addAll(Arrays.asList(array));
    }

    /**
     * Appends all of a Point array at a certain index.
     * Adds the first Point at the specified index.
     * all Points at an index greater than the specified index get moved to after the array.
     *
     * @param index the index to add the Point array (0 to place array at start)
     * @param array the array of points to add.
     */
    public void addAll(int index, Waypoint[] array) {
        if (!(index < path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException();
        path.addAll(index % path.size(), Arrays.asList(array));
    }

    /**
     * Clears the path
     */
    public void clear() {
        path.clear();
    }

    /**
     * /**
     * Returns the size of the path.
     *
     * @return returns the size() of the array.
     */
    public int length() {
        return path.size();
    }

    /**
     * Copies the path
     *
     * @return
     */
    public Path copy() {
        return new Path(path);
    }

    // ----== Functions for path generation and optimisation: ==----

    /**
     * Converts the path ArrayList to an array.
     *
     * @param array needed to specify what type of array will copy over.
     * @return returns a Waypoint[] array.
     */
    public Waypoint[] toArray(Waypoint[] array) {
        return path.toArray(array);
    }

    /**
     * Adds points at a certain spacing between them into all the segments.
     */
    public Path generateFillPoint() {
        //double vector = Point.distance(path.get(0), path.get(path.size() - 1));
        //final int NUM_OF_POINTS_THAT_CAN_FIT = (int) Math.ceil(vector / Constants.SPACING_BETWEEN_WAYPOINTS);

        Vector[] pathVectors = new Vector[path.size()]; //create an array of vectors per point.
        Path newPoints = new Path(); //create a new path class
        int AmountOfPoints;
        for (int i = 0; i < pathVectors.length - 1; i++) {//for every point on the path
            //Creates a vector with the slope of the two points we are checking
            //Sets the vectors magnitude to the constant of the spacing
            //calculates the amount of fill-points that can fit between the two points.
            pathVectors[i] = new Vector(this.getWaypoint(i), this.getWaypoint(i + 1));
            AmountOfPoints = (int) Math.ceil(pathVectors[i].magnitude() / Constants.SPACING_BETWEEN_WAYPOINTS);
            pathVectors[i] = pathVectors[i].normalize().multiply(Constants.SPACING_BETWEEN_WAYPOINTS);
            for (int j = 0; j < AmountOfPoints; j++) {
                newPoints.appendWaypoint(pathVectors[i].multiply(j).add(this.getWaypoint(i)));
            }
        }
        return newPoints;


    }

    /**
     * @param weight_data   amount of data
     * @param weight_smooth amount of smooth
     * @param tolerance     the min change between points
     * @return the new path with the way points
     * @author Paulo
     * @author Lior
     */
    public Path generateSmoothing(double weight_data, double weight_smooth, double tolerance) {
        Path newPathClass = this.copy();
        double[][] newPath = new double[this.length()][2];
        double a = weight_data;
        double b = weight_smooth;
        for (int i = 0; i < this.length(); i++) {
            newPath[i][0] = this.getWaypoint(i).getX();
            newPath[i][1] = this.getWaypoint(i).getY();

        }
        double[][] path = doubleArrayCopy(newPath);
        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++)
                for (int j = 0; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += a * (path[i][j] - newPath[i][j]) + b *
                            (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }
        for (int i = 0; i < this.length(); i++) {
            Waypoint p = newPathClass.getWaypoint(i);
            p.setX(newPath[i][0]);
            p.setY(newPath[i][1]);
            newPathClass.setWaypoint(i, p);
        }
        return newPathClass;
    }

    /**
     * Attributes to all points their distance from the start.
     */
    private void generateDistance() {
        this.recursiveDistance(this.length() - 1);
    }

    /**
     * Returns the size of the largest length in the list.
     *
     * @param i index of current point
     * @return returns sum of all distances before this point.
     */
    private double recursiveDistance(int i) {
        if (i == 0)
            return 0;
        double d = recursiveDistance(i - 1) + Point.distance(path.get(i), path.get(i - 1));
        Waypoint p = path.get(i);
        p.setDistance(d);
        path.set(i, p);
        return d;
    }

    /**
     * Attributes to all points their curvature in correlation to their adjacent points.
     */
    public void generateCurvature() {
        double k1, k2, b, a, r;
        for (int i = 1; i < path.size() - 1; i++) {
            double x1 = path.get(i).getX();
            if (path.get(i - 1).getX() == x1)
                x1 += 0.0001;
            k1 = 0.5 * (Math.pow(x1, 2) + Math.pow(path.get(i).getY(), 2) - Math.pow(path.get(i - 1).getX(), 2) - Math.pow(path.get(i - 1).getY(), 2)) / (x1 - path.get(i - 1).getX());
            k2 = (path.get(i).getY() - path.get(i - 1).getY()) / (x1 - path.get(i - 1).getX());
            b = 0.5 * (Math.pow(path.get(i - 1).getX(), 2) - 2 * path.get(i - 1).getX() * k1 + Math.pow(path.get(i - 1).getY(), 2) - Math.pow(path.get(i + 1).getX(), 2) + 2 * path.get(i + 1).getX() * k1 - Math.pow(path.get(i + 1).getY(), 2)) / (path.get(i + 1).getX() * k2 - path.get(i + 1).getY() + path.get(i - 1).getY() - path.get(i - 1).getX() * k2);
            a = k1 - k2 * b;
            r = Math.sqrt(Math.pow(x1 - a, 2) + Math.pow(path.get(i).getY() - b, 2));
            double curv = 0;
            if (r == 0) {
                curv = Double.POSITIVE_INFINITY;
            } else {
                curv = 1 / r;
            }
            path.get(i).setCurvature(curv);
        }
    }


    /**
     * @param const_acceleration rhe acceleration constant
     * @author orel
     * @author orel
     */
    public void generateVelocity(double const_acceleration) {
        double maximum_velocity;
//accurate calculation
        for (int i = 1; i < this.length() - 1; i++) {
            maximum_velocity = Math.min(Math.sqrt(2 * const_acceleration * Point.distance(this.getWaypoint(i), this.getWaypoint(i-1)) + Math.pow(this.getWaypoint(i - 1).getSpeed(), 2)), const_acceleration / this.getWaypoint(i).getCurvature());
            this.getWaypoint(i).setSpeed(maximum_velocity);
        }

        for (int i = this.length() - 2; i > 0; i--) {
            this.getWaypoint(i).setSpeed(Math.min(this.getWaypoint(i).getSpeed(), Math.sqrt(Math.pow(this.getWaypoint(i + 1).getSpeed(), 2) + 2 * const_acceleration * Point.distance(this.getWaypoint(i), this.getWaypoint(i-1)))));
        }

    }

    @Override
    public String toString() {
        return "Path{" + "path=" + path + '}';
    }
}

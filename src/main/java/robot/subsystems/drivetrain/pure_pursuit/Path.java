package robot.subsystems.drivetrain.pure_pursuit;

import robot.subsystems.drivetrain.Constants;
import java.math.*;
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

    /**
     * Set a point at an index.
     *
     * @param index index of the desired point starting at zero, use -1 for last Point.
     */
    public void set(int index, Waypoint p) {
        if (!(index < path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException();
        path.set(index % path.size(), p);
    }

    /**
     * Adds point to the path.
     *
     * @param index the index of the point to append.
     * @param p     the waypoint to add.
     */
    public void add(int index, Waypoint p) {
        if (!(index < path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException();
        path.add(index % path.size(), p);
    }

    /**
     * Appends an object to the end of the list.
     *
     * @param p the waypoint to add.
     */
    public void append(Waypoint p) {
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
     * get a Point at a specific index.
     *
     * @param index index of the desired point starting at zero, use -1 for last Point.
     * @return returns the Point.
     */
    public Point get(int index) {
        if (!(index < path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException();
        if (path.get(index % path.size()) == null)
            throw new ClassCastException("Tried to call a non Point object from the path list.");
        return path.get(index % path.size());
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
     * @return
     */
    public Path copy(){
        return new Path(path);
    }
    /**
     * Converts the path ArrayList to an array.
     *
     * @param array needed to specify what type of array will copy over.
     * @return returns a Waypoint[] array.
     */
    public Waypoint[] toArray(Waypoint[] array) {
        return path.toArray(array);
    }

    // ----== Functions for path generation and optimisation: ==----

    /**
     * Adds points at a certain spacing between them into all the segments.
     */
    private void generateFillPoint() {
        double vector = Point.distance(path.get(0), path.get(path.size() - 1));
        final int NUM_OF_POINTS_THAT_CAN_FIT = (int) Math.ceil(vector / Constants.SPACING_BETWEEN_WAYPOINTS);

    }


    /**
     * @author Paulo
     * @author Lior
     * @param weight_data amount of data
     * @param weight_smooth amount of smooth
     * @param tolerance the min change between points
     * @return the new path with the way points
     */
    private Path genrate_smoothing(double weight_data, double weight_smooth, double tolerance) {
        Path newPathClass = this.copy();
        double[][] newPath = new double[this.length()][2];
        double a = weight_data;
        double b = weight_smooth;
        for(int i = 0; i<this.length(); i++){
            newPath[i][0] = this.getWaypoint(i).getX();
            newPath[i][1] = this.getWaypoint(i).getY();

        }
        double[][] path = doubleArrayCopy(newPath);
        double change = tolerance;
        while(change >= tolerance) {
            change = 0.0;
            for(int i=1; i<path.length-1; i++)
                for(int j=0; j<path[i].length; j++) {
                    double aux = newPath[i][j]; newPath[i][j] += a * (path[i][j] - newPath[i][j]) + b *
                        (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
            }
        }
        for(int i = 0; i<this.length(); i++){
            Waypoint p = newPathClass.getWaypoint(i);
            p.setX(newPath[i][0]);
            p.setY(newPath[i][1]);
            newPathClass.set(i, p);
        }
        return newPathClass;
    }

    public static double[][] doubleArrayCopy(double[][] arr)
    {
        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];
        for(int i=0; i<arr.length; i++)
        {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];
            //Copy Contents
            for(int j=0; j<arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }
        return temp;
    }

    /**
     * Attributes to all points their distance from the start.
     */
    private void generateDistance() {

    }

    /**
     * Attributes to all points their curvature in correlation to their adjacent points.
     */
    private void generateCurvature() {

    }


    /**
     * @author orel
     * @param path the path
     * @param const_acceleration rhe acceleration constant
     */
    private void generateVelocity(Path path,double const_acceleration ) {
        int constant_for_velocity = 2;
        double maximum_velocity_simplified;
        double maximum_velocity;
//accurate calculation
        for (int i = 1; i < path.length() - 1; i++) {
            maximum_velocity = Math.sqrt(2 * const_acceleration * Constants.SPACING_BETWEEN_WAYPOINTS + Math.pow(path.getWaypoint(i).getSpeed(), 2));
            path.getWaypoint(i).setSpeed(maximum_velocity);
        }

//simplified calculation
        for (int i =1; i< path.length()-1; i++) {
            maximum_velocity_simplified = constant_for_velocity / path.getWaypoint(i).curvature;
        }


    }


}

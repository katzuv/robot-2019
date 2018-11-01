package robot.subsystems.drivetrain.pure_pursuit;

import robot.subsystems.drivetrain.Constants;

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
    private Path generateFillPoint() {
        double vector = Point.distance(path.get(0), path.get(path.size() - 1));
        final int NUM_OF_POINTS_THAT_CAN_FIT = (int) Math.ceil(vector / Constants.SPACING_BETWEEN_WAYPOINTS);

        Vector[] pathVectors = new Vector[path.size()];
        Path newPoints = new Path();
        int AmountOfPoints;
        for (int i = 0; i < pathVectors.length-1; i++) {
            pathVectors[i] = new Vector(path.get(i), path.get(i + 1));
            AmountOfPoints = (int) Math.ceil(pathVectors[i].magnitude() / Constants.SPACING_BETWEEN_WAYPOINTS);
            pathVectors[i] = pathVectors[i].normalize().multiply(Constants.SPACING_BETWEEN_WAYPOINTS);
            for (int j = 0; j < AmountOfPoints; j++) {
                    newPoints.append(pathVectors[i].multiply(j).addWaypoint(this.getWaypoint(i)));
            }
        }
        return newPoints;



    }

    /**
     * Takes all of the points and makes the curve smoother.
     */
    private void generateSmoothing() {

    }

    /**
     * Attributes to all points their distance from the start.
     */
    private void generateDistance() {
        this.recursiveDistance(this.length());
    }

    /**
     * Returns the size of the largest length in the list.
     * @param i index of current point
     * @return returns sum of all distances before this point.
     */
    private double recursiveDistance(int i){
        if(i==0)
            return 0;
        double d = recursiveDistance(i-1) + Point.distance(path.get(i),path.get(i-1));
        Waypoint p = path.get(i);
        p.setDistance(d);
        path.set(i, p);
        return d;
    }

    /**
     * Attributes to all points their curvature in correlation to their adjacent points.
     */
    private void generateCurvature() {
        double k1, k2, b, a, r;
        for (int i = 1; i < path.size()-1;i++)
        {
            double x1 = path.get(i).getX();
            if(path.get(i-1).getX() == x1)
                x1 += 0.0001;
            k1 = 0.5*(Math.pow(x1, 2) + Math.pow(path.get(i).getY(), 2) - Math.pow(path.get(i-1).getX(), 2) - Math.pow(path.get(i-1).getY(), 2))/(x1-path.get(i-1).getX());
            k2 = (path.get(i).getY() - path.get(i-1).getY())/(x1 - path.get(i-1).getX());
            b = 0.5 * (Math.pow(path.get(i-1).getX(), 2) - 2 * path.get(i-1).getX() * k1 + Math.pow(path.get(i-1).getY(),2) - Math.pow(path.get(i+1).getX(), 2) + 2 * path.get(i+1).getX() * k1 - path.get(i+1).getY())/(path.get(i+1).getX()*k2 - path.get(i+1).getY() + path.get(i-1).getY() - path.get(i-1).getX() * k2);
            a = k1 - k2 * b;
            r = Math.sqrt(Math.pow(x1 - a,2) + Math.pow(path.get(i).getY() - b,2));
            double curv = 0;
            if(r==0){
                curv = Double.POSITIVE_INFINITY;
            }
            else{
                curv = 1 / r;
            }
            path.get(i).setCurvature(curv);
        }
    }

    /**
     * Arrributes to all points their intended velocity.
     */
    private void generateVelocity() {

    }


}

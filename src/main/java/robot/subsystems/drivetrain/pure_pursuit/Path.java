package robot.subsystems.drivetrain.pure_pursuit;

import robot.subsystems.drivetrain.Constants;

import java.lang.reflect.Array;
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

    public Path(ArrayList<Waypoint> w){
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
     *
     * @param index
     * @param p
     */
    public void add(int index, Waypoint p){
        if (!(index < path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException();
        path.add(index % path.size(), p);
    }

    /**
     *
     * @param start
     * @param length
     * @return
     */
    public Path get(int start, int length){
        ArrayList p = path;
        for(int i = 0; i<path.size(); i++){
            if(i<start || i > start+length)
                p.remove(i);
        }
        return new Path(p);
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

    public void clear(){
        path.clear();
    }

    public double length(){
        return path.size();
    }

    /**
     * Converts the path ArrayList to an array.
     * @param array needed to specify what type of array will copy over.
     * @return returns a Waypoint[] array.
     */
    public Waypoint[] toArray(Waypoint[] array){
        return path.toArray(array);
    }

    // ----== Functions for path generation and optimisation: ==----

    /**
     * Adds points at a certain spacing between them into all the segments
     */
    private void generateFillPoint() {
        double vector = Point.distance(path.get(0), path.get(path.size() - 1));
        final int NUM_OF_POINTS_THAT_CAN_FIT = (int) Math.ceil(vector / Constants.SPACING_BETWEEN_WAYPOINTS);

    }

    /**
     *
     */
    private void generateSmoothing() {

    }

    /**
     *
     */
    private void generateDistance() {

    }

    /**
     *
     */
    private void generateCurvature() {

    }

    /**
     *
     */
    private void generateVelocity() {

    }


}

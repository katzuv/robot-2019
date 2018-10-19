package robot.subsystems.drivetrain.pure_pursuit;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * @author Paulo Khayat
 * @author Lior Barkai
 * This class is the instance of the path, holding the points.
 */
public class Path {
    private ArrayList<Point> path = new ArrayList<Point>();

    /**
     * Create an empty Path instance
     */
    public Path(){

    }

    /**
     * Create a Path instance
     * @param array the array of points to add into the arraylist
     */
    public Path(Point[] array){
        path.addAll(Arrays.asList(array));
    }
    /**
     * Adds all of a Point array to the end of the path list
     * @param array the array of points to add to the end of the array.
     */
    public void addAll(Point[] array){
        path.addAll(Arrays.asList(array));
    }

    /**
     * Appends all of a Point array at a certain index.
     * Adds the first Point at the specified index.
     * all Points at an index greater than the specified index get moved to after the array.
     * @param index the index to add the Point array (0 if you want the point array to be first in line.
     * @param array the array of points to add.
     */
    public void addAll(int index, Point[] array){
        path.addAll(index, Arrays.asList(array));
    }

    /**
     * get a Point at a specific index.
     * @param index index of the point starting at zero.
     * @return returns the Point.
     */
    public Point get(int index){
        if(path.get(index) == null)
            throw new ClassCastException("Tried to call a non Point object from the path list.");
        return path.get(index);
    }


}

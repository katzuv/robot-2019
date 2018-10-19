package robot.subsystems.drivetrain.pure_pursuit;

import java.util.ArrayList;
import java.util.Arrays;

/**
 *
 */
public class Path {
    private ArrayList<Point> path = new ArrayList<Point>();

    /**
     *
     */
    public void addAll(Point[] array)
    {
        path.addAll(Arrays.asList(array));
    }

    public void addAll(int index, Point[] array) {
        for (int i = 0; i < array.length; i++)
            path.add(index + i, array[i]);
    }

    public Point get(int index)
    {
        if (path.get(index) == null)
            throw new NullPointerException("Tried to call a null point");
        return path.get(index);
    }
    public void generatePoints()
    {

    }

}

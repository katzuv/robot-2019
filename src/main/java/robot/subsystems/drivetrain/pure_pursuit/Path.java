package robot.subsystems.drivetrain.pure_pursuit;

import java.util.ArrayList;
/**
 *
 */
public class Path {
    private ArrayList path = new ArrayList();

    /**
     *
     */
    public void addAll(Point[] array){
        for(int i = 0; i<array.length; i++)
            path.add(array[i]);
    }

    public void addAll(int index, Point[] array){
        for(int i = 0; i<array.length; i++)
            path.add(index+i,array[i]);
    }

    public Point get(int index){
        if(!(path.get(index) instanceof Point))
            throw new ClassCastException("Tried to call a non Point object from the path list.");
        return (Point) path.get(index);
    }


}

package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.drive.Vector2d;
import org.jetbrains.annotations.NotNull;

public class Vector extends Vector2d {
    public Vector(){
        super();
    }

    public Vector(double x, double y){
        super(x,y);
    }

    public Vector(@NotNull Point start,@NotNull Point end){
        super(end.getX() - start.getX(), end.getY() - start.getY());
    }
    
    public Vector add(Vector2d vec){
        return new Vector(x + vec.x, y + vec.y);
    }

    public Vector subtract(Vector2d vec){
        return new Vector(x - vec.x, y - vec.y);
    }

}


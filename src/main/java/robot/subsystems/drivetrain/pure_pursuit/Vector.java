package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class Vector extends Vector2d {
    public Vector() {
        super();
    }

    public Vector(double x, double y) {
        super(x, y);
    }

    public Vector(Point start, Point end) {
        super(end.getX() - start.getX(), end.getY() - start.getY());
    }

    public Vector add(Vector2d vec) {
        return new Vector(x + vec.x, y + vec.y);
    }

    public Point add(Point p) {
        return new Point(p.getX() + x, p.getY() + y);
    }

    public Waypoint add(Waypoint p){
        return new Waypoint(p.getX()+x, p.getY()+y, p.getDistance(), p.getSpeed(), p.getCurvature());
    }

    public Vector subtract(Vector2d vec){
        return new Vector(x - vec.x, y - vec.y);
    }

    public Waypoint subtract(Waypoint p){
        return new Waypoint(p.getX()-x, p.getY()-y, p.getDistance(), p.getSpeed(), p.getCurvature());
    }

    public Vector multiply(double d){
        return new Vector(x*d, y*d);
    }



    public Vector normalize() {
        return this.multiply(1 / this.magnitude());
    }
}


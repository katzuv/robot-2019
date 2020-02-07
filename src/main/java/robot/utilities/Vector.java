package robot.utilities;

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


    public Vector subtract(Vector2d vec) {
        return new Vector(x - vec.x, y - vec.y);
    }


    public Vector multiply(double d) {
        return new Vector(x * d, y * d);
    }

    public double angle() {
        return Math.toDegrees(Math.atan2(y, x));
    }

    public Vector normalize() {
        return this.multiply(1 / this.magnitude());
    }

    public Vector rotated(double angle) {
        double radians = -Math.toRadians(angle);
        return new Vector(x * Math.cos(radians) - y * Math.sin(radians), x * Math.sin(radians) + y * Math.cos(radians));
    }

    public String toString() {
        return String.format("x: %f, y: %f, d: %f", x, y, angle());
    }
}



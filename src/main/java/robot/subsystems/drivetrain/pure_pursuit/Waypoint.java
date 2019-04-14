package robot.subsystems.drivetrain.pure_pursuit;


import robot.utilities.Point;

public class Waypoint extends Point {
    private double distance, speed, curvature;

    public Waypoint(double x, double y, double distance, double speed, double curvature) {
        super(x, y);
        this.speed = speed;
        this.distance = distance;
        this.curvature = curvature;
    }

    public Waypoint(double x, double y) {
        super(x, y);
    }

    public Waypoint(Point p){ super(p.getX(), p.getY());}


    public Waypoint(Waypoint p) {
        super(p.getX(), p.getY());
        this.speed = p.getSpeed();
        this.distance = p.getDistance();
        this.curvature = p.getCurvature();
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getCurvature() {
        return curvature;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    @Override
    public String toString() {
        return "" + "distance=" + distance + ", \t speed=" + speed + ", \t curvature=" + curvature + ",\t x=" + x + ",\t  y=" + y + "\n";
    }


    public Waypoint copy() {
        return new Waypoint(this);
    }
}

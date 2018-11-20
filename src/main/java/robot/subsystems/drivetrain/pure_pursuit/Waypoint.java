package robot.subsystems.drivetrain.pure_pursuit;

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


    public Waypoint(Waypoint p) {
        super(p.getX(), p.getY());
        this.speed = p.getSpeed();
        this.distance = p.getDistance();
        this.curvature = p.getCurvature();
    }

    public static Waypoint rotate(Point center, Waypoint p, double degrees) {
        double radians = -Math.toRadians(degrees);
        Waypoint newWaypoint = new Waypoint(p);
        newWaypoint.setX(center.getX() + (p.getX() - center.getX()) * Math.cos(radians) - (p.getY() - center.getY()) * Math.sin(radians));
        newWaypoint.setX(center.getY() + (p.getX() - center.getX()) * Math.sin(radians) + (p.getY() - center.getY()) * Math.cos(radians));
        return newWaypoint;

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

    public double getCurvature() { return curvature;}

    public void setCurvature(double curvature) { this.curvature = curvature;}

    @Override
    public String toString() {
        return "" + "distance=" + distance + ", \t speed=" + speed + ", \t curvature=" + curvature + ",\t x=" + x + ",\t  y=" + y + "\n";
    }
    public static Waypoint rotate(Point center, Waypoint p, double degrees) {
        double radians = -Math.toRadians(degrees);
        Waypoint newWaypoint = new Waypoint(p);
        newWaypoint.setX(center.getX() + (p.getX() - center.getX()) * Math.cos(radians) - (p.getY() - center.getY()) * Math.sin(radians));
        newWaypoint.setX(center.getY() + (p.getX() - center.getX()) * Math.sin(radians) + (p.getY() - center.getY()) * Math.cos(radians));
        return newWaypoint;

    }

    public Waypoint copy() {
        return new Waypoint(this);
    }
}

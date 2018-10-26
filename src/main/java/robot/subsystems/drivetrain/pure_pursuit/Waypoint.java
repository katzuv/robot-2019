package robot.subsystems.drivetrain.pure_pursuit;

public class Waypoint extends Point {
    protected double distance, speed, curvature;

    public Waypoint(double x, double y, double distance, double speed, double curvature) {
        super(x, y);
        this.speed = speed;
        this.distance = distance;
    }
    public Waypoint(double x, double y){
        super(x, y);
    }

    public Waypoint(Point p){
        super(p.getX(), p.getY());
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

}

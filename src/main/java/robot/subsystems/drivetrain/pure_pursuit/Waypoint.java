package robot.subsystems.drivetrain.pure_pursuit;

public class Waypoint extends Point {
    public double distance;
    public double speed;
    public double curvature;

    public Waypoint(double x, double y, double distance, double speed, double curvature) {
        super(x, y);
        this.speed = speed;
        this.distance = distance;
        this.curvature = curvature;
    }
    public Waypoint(double x, double y){
        super(x, y);
    }

    public Waypoint(Point p){
        super(p.getX(), p.getY());
    }

    public Waypoint(Waypoint p){
        super(p.getX(),p.getY());
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

    public double getCurvature() { return curvature;}

    public void setCurvature(double curvature) { this.curvature = curvature;}

    @Override
    public String toString() {
        return "" +
   //             "distance=" + distance +
    //            ", speed=" + speed +
                ", curvature=" + curvature +
                ", x=" + x +
                ", y=" + y +
                  "\n";
    }
}

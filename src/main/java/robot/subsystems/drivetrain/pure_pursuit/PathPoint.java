package robot.subsystems.drivetrain.pure_pursuit;

public class PathPoint extends Point {
    protected double distance, speed;

    public PathPoint(double x, double y, double distance, double speed) {
        super(x, y);
        this.speed = speed;
        this.distance = distance;
    }
    public PathPoint(double x, double y){
        super(x, y);
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
}

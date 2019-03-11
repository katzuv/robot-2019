package robot.subsystems.drivetrain.pure_pursuit;

public class Point3D extends Point {
    protected double z;

    public Point3D(double x, double y, double z){
        super(x, y);
        this.z = z;
    }

    public static double distance(Point3D p1, Point3D p2) {
        return Math.hypot(Math.hypot(p2.getX() - p1.getX(), p2.getY() - p1.getY()), p2.getZ() - p1.getZ());
    }

    public static Point3D average(Point3D p1, Point3D p2) {
        return new Point3D(0.5 * (p1.getX() + p2.getX()), 0.5 * (p1.getY() + p2.getY()), 0.5 * (p1.getZ() + p2.getZ()));
    }

    public double getZ(){
        return z;
    }

    public void setZ(double z){
        this.z = z;
    }

    public double magnitude(){
        return distance(new Point3D(0,0,0), this);
    }
}

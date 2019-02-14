package robot.subsystems.drivetrain.pure_pursuit;

public class Point3D extends Point {
    protected double z;

    public Point3D(double x, double y, double z){
        super(x, y);
        this.z = z;
}

}
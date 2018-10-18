package robot.subsystems.drivetrain.pure_pursuit;

/**
 * A generic point class for storing (x, y) coordinates
 * @author Paulo Khayat
 */
public class Point {
    private double x, y;

    /**
     * Gets the two coordinates of the point
     * @param x: the x coordinate of the point
     * @param y: the y coordinate of the point
     */
    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    /**
     * @return returns the X coordinate
     */
    public double getX() {
        return x;
    }

    /**
     * @param x sets the x coordinate
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * @return returns the Y coordinate
     */
    public double getY() {
        return y;
    }

    /**
     * @param y sets the Y coordinate
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * @param p1 The first Point object reference
     * @param p2 The second Point object reference
     * @return returns the distance between the two points
     */
    public static double distance(Point p1, Point p2){
        return Math.hypot(p2.getX()-p1.getX(), p2.getY() - p1.getY());
    }

    /**
     *
     * @param p1 The first Point object reference
     * @param p2 The second Point object reference
     * @return returns a new Point which is the average of the two given points
     */
    public static Point average(Point p1, Point p2){
        return new Point(0.5*(p1.getX()+p2.getX()), 0.5*(p1.getY()+p2.getY()));
    }
}

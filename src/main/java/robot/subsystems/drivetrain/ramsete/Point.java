package robot.subsystems.drivetrain.ramsete;

public class Point {
    protected double x, y, angle;

    /**
     * Gets the two coordinates of the point
     *
     * @param x: the x coordinate of the point
     * @param y: the y coordinate of the point
     */
    public Point(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
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
     * @return returns the angle
     */
    public double getAngle() {
        return angle;
    }


    /**
     * @param y sets the Y coordinate
     */
    public void setY(double y) {
        this.y = y;
    }

    public Point subtract(Point point) {
        return new Point(point.getX() - x, point.getY() - y, point.getAngle() - angle);
    }

}


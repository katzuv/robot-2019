package robot.subsystems.drivetrain.pure_pursuit;


import java.util.ArrayList;
import java.util.Arrays;


/**
 * @author Paulo Khayat
 * @author Lior Barkai
 * <p>
 * This class is the instance of the path, holding the points.
 * The generation methods written here are all part of the Pure pursuit algorithm
 * all instances of the name 'the pure pursuit article' refer to this article by team DAWGMA 1712:
 * https://www.chiefdelphi.com/media/papers/download/5533
 */
public class Path {
    private ArrayList<Waypoint> path = new ArrayList<>();

    /**
     * Create an empty Path instance
     */
    public Path() {

    }

    /**
     * Create a Path instance
     *
     * @param array the array of points to add into the arraylist
     */
    public Path(Waypoint[] array) {
        path.addAll(Arrays.asList(array));
    }

    public Path(ArrayList<Waypoint> w) {
        path.addAll(w);
    }

    private static double[][] doubleArrayCopy(double[][] arr) {
        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];
        for (int i = 0; i < arr.length; i++) {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];
            //Copy Contents
            for (int j = 0; j < arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }
        return temp;
    }

    /**
     * Set a point at an index.
     *
     * @param index index of the desired point starting at zero, use -1 for last Point.
     */
    public void setWaypoint(int index, Waypoint p) {
        if (!(index <= path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException("Waypoint index " + index + " is out of bounds.");
        if (index == path.size()) //if set is just out of bounds the method appends the waypoint instead.
            this.appendWaypoint(p);
        else
            path.set(Math.floorMod(index, path.size()), p);
    }

    /**
     * Add a point to the path.
     *
     * @param index the index of the point to append.
     * @param p     the waypoint to add.
     */
    public void addWaypoint(int index, Waypoint p) {
        if (!(index <= path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException("Waypoint index " + index + " is out of bounds.");
        if (path.get(Math.floorMod(index, path.size())) == null)
            throw new ClassCastException("Tried to call a non Point object from the path list.");
        if (index == path.size()) //if set is just out of bounds the method appends the waypoint instead.
            this.appendWaypoint(p);
        else
            path.add(Math.floorMod(index, path.size()), p);
    }

    /**
     * Append an object to the end of the list.
     *
     * @param p the waypoint to add.
     */
    public void appendWaypoint(Waypoint p) {
        path.add(p);
    }

    /**
     * Return a new subPath of the path.
     *
     * @param start  first index of the path.
     * @param length last index (subtracted by one).
     * @return returns a new Path class which holds the specified sublist of waypoints.
     */
    public Path getSubPath(int start, int length) {
        return new Path(new ArrayList<>(path.subList(start, length)));
    }

    /**
     * Return a Waypoint at a specific index.
     *
     * @param index index of the desired point starting at zero, use -1 for last Point.
     * @return returns the Point.
     */
    public Waypoint getWaypoint(int index) {
        if (!(index < path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException("Waypoint index " + index + " is out of bounds.");
        return path.get(Math.floorMod(index, path.size()));
    }

    /**
     * Add all of a Point array to the end of the path list.
     * The equivalent of 'addAll(-1, array)'
     *
     * @param array the array of points to add to the end of the array.
     */
    public void addAll(Waypoint[] array) {
        path.addAll(Arrays.asList(array));
    }

    /**
     * Append all of a Point array at a certain index.
     * Adds the first Point at the specified index.
     * all Points at an index greater than the specified index get moved to after the array.
     *
     * @param index the index to add the Point array (0 to place array at start)
     * @param array the array of points to add.
     */
    public void addAll(int index, Waypoint[] array) {
        if (!(index < path.size() && index > -path.size()))
            throw new ArrayIndexOutOfBoundsException("Waypoint index " + index + " is out of bounds.");
        path.addAll(Math.floorMod(index, path.size()), Arrays.asList(array));
    }

    /**
     * Adds all of a Point array to the end of the path list.
     * The equivalent of 'addAll(-1, array)'
     *
     * @param path the path class of points to add to the end of the array.
     */
    public void addAll(Path path) {
        addAll(path.toArray());
    }

    /**
     * Appends all of a Point array at a certain index.
     * Adds the first Point at the specified index.
     * all Points at an index greater than the specified index get moved to after the array.
     *
     * @param index the index to add the Point array (0 to place array at start)
     * @param path  the path class of points to add.
     */
    public void addAll(int index, Path path) {
        addAll(index, path.toArray());
    }

    /**
     * Clears the path
     */
    public void clear() {
        path.clear();
    }

    /**
     * /**
     * Return the size of the path.
     *
     * @return returns the size() of the array.
     */
    public int length() {
        return path.size();
    }

    /**
     * Create a new instance of the path.
     *
     * @return
     */
    public Path copy() {
        return new Path(path);
    }

    // ----== Functions for path generation and optimisation: ==----

    /**
     * Convert the path ArrayList to an array.
     *
     * @return returns a Waypoint[] array.
     */
    public Waypoint[] toArray() {
        return path.toArray(new Waypoint[]{});
    }

    /**
     * Run all generate methods at once.
     *
     * @param weight_data        generateSmoothing parameter. See documentation of the generateSmoothing method for more information.
     * @param weight_smooth      generateSmoothing parameter. See documentation of the generateSmoothing method for more information.
     * @param tolerance          generateSmoothing parameter. See documentation of the generateSmoothing method for more information.
     * @param const_acceleration generateVelocities parameter. See documentation of the generateVelocities method for more information.
     */
    public void generateAll(double weight_data, double weight_smooth, double tolerance, double const_acceleration, double max_path_velocity) {
        this.generateFillPoint();
        this.generateSmoothing(weight_data, weight_smooth, tolerance);
        this.generateCurvature();
        this.generateDistance();
        this.generateVelocity(const_acceleration, max_path_velocity);
    }

    /**
     * Add points at a certain spacing between them into all the segments.
     * <p>
     * The first of the five methods used in the path generation, needed for the pure pursuit.
     * (Pure pursuit article, 'Path Generation' > 'Injecting points' , Page 5)
     */
    public void generateFillPoint() {
        Vector[] pathVectors = new Vector[path.size()]; //create an array of vectors per point.
        Path newPathClass = new Path(); //create a new path class
        int AmountOfPoints;
        for (int i = 0; i < pathVectors.length - 1; i++) {//for every point on the path
            //Creates a vector with the slope of the two points we are checking
            //Sets the vectors magnitude to the constant of the spacing
            //calculates the amount of fill-points that can fit between the two points.
            pathVectors[i] = new Vector(this.getWaypoint(i), this.getWaypoint(i + 1));
            AmountOfPoints = (int) Math.ceil(pathVectors[i].magnitude() / Constants.SPACING_BETWEEN_WAYPOINTS);
            pathVectors[i] = pathVectors[i].normalize().multiply(Constants.SPACING_BETWEEN_WAYPOINTS);
            for (int j = 0; j < AmountOfPoints; j++) {
                newPathClass.appendWaypoint(pathVectors[i].multiply(j).add(this.getWaypoint(i)));
            }
        }
        clear();
        addAll(newPathClass);


    }

    /**
     * Smooth the points in the path class.
     * <p>
     * The second of the five methods used in the path generation, needed for the pure pursuit.
     * (Pure pursuit article, 'Path Generation' > 'Smoothing' , Page 5)
     *
     * @param weight_data   smooth constant. controls the proportional distance from the original point.
     * @param weight_smooth smooth constant. controls the proportional distance from the midpoint.
     * @param tolerance     the minimum change between points.
     * @return the new path with the way points.
     * @author Paulo
     * @author Lior
     */
    public void generateSmoothing(double weight_data, double weight_smooth, double tolerance) {
        /*
         * For simplification and parallelism with the article with the pure pursuit article, we converted the Waypoint
         * path into a matrix of doubles. the first dimension represents the index of the waypoint, the second dimension
         * the first column (column [0]) holding the x values and the second column (column [1]) holding the Y values.
         *
         * this smoothing algorithm takes each point on the path, tries to move it towards the midpoint of the next and
         * previous point, proportionally to the distance from the midpoint, minus the amount it already moved until now.
         * the program ends when the amount of change passes the tolerance.
         */
        Path newPathClass = this.copy();
        double[][] newPath = new double[this.length()][2];
        for (int i = 0; i < this.length(); i++) { //copying the path to an array.
            newPath[i][0] = this.getWaypoint(i).getX();
            newPath[i][1] = this.getWaypoint(i).getY();

        }
        double[][] path = doubleArrayCopy(newPath);
        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++)
                for (int j = 0; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth *
                            (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }
        for (int i = 0; i < this.length(); i++) {
            Waypoint p = newPathClass.getWaypoint(i);
            p.setX(newPath[i][0]);
            p.setY(newPath[i][1]);
            newPathClass.setWaypoint(i, p);
        }
        this.clear();
        this.addAll(newPathClass);
    }

    /**
     * Attribute to all points their distance from the start.
     * Set the distance of each point from the start and save as a parameter, so that it doesn't have to be calculated real time.
     * <p>
     * The third of the five methods used in the path generation, needed for the Pure Pursuit.
     * (see the Pure pursuit article, 'Path Generation' > 'Distances Between Points' , Page 6)
     */
    public void generateDistance() {
        this.recursiveDistance(this.length() - 1); //uses a recursive method to attribute each distance from the start.
    }

    /**
     * Return the size of the largest length in the list.
     *
     * @param i index of current point
     * @return returns sum of all distances before this point.
     */
    private double recursiveDistance(int i) {
        if (i == 0)
            return 0;
        double d = recursiveDistance(i - 1) + Point.distance(path.get(i), path.get(i - 1));
        Waypoint p = path.get(i);
        p.setDistance(d);
        path.set(i, p);
        return d;
    }

    /**
     * Attribute to all points their curvature in correlation to their adjacent points.
     * set the curvature of each point and save as a parameter, so that it doesn't have to be calculated real time.
     * <p>
     * The fourth of the five methods used in the path generation, needed for the Pure Pursuit.
     * (see the Pure pursuit article, 'Path Generation' > 'Curvature of Path' , Page 6)
     */
    public void generateCurvature() {
        /*
         * For each point on the path:
         * (a, b) are the center of the circle that intersects with the point, its previous point and its next point.
         * r is the radius of that given circle.
         * k1 and k2 are used to find the center of the circle.
         */
        double k1, k2, b, a, r;
        for (int i = 1; i < path.size() - 1; i++) {
            double x1 = path.get(i).getX();
            if (path.get(i - 1).getX() == x1)
                x1 += 0.0001;
            k1 = 0.5 * (Math.pow(x1, 2) + Math.pow(path.get(i).getY(), 2) - Math.pow(path.get(i - 1).getX(), 2) - Math.pow(path.get(i - 1).getY(), 2)) / (x1 - path.get(i - 1).getX());
            k2 = (path.get(i).getY() - path.get(i - 1).getY()) / (x1 - path.get(i - 1).getX());
            b = 0.5 * (Math.pow(path.get(i - 1).getX(), 2) - 2 * path.get(i - 1).getX() * k1 + Math.pow(path.get(i - 1).getY(), 2) - Math.pow(path.get(i + 1).getX(), 2) + 2 * path.get(i + 1).getX() * k1 - Math.pow(path.get(i + 1).getY(), 2)) / (path.get(i + 1).getX() * k2 - path.get(i + 1).getY() + path.get(i - 1).getY() - path.get(i - 1).getX() * k2);
            a = k1 - k2 * b;
            r = Math.sqrt(Math.pow(x1 - a, 2) + Math.pow(path.get(i).getY() - b, 2));
            double curv = 0;
            if (r == 0) { //if the radius is zero, we would get a zero division error.
                curv = Math.pow(10, 6);
            } else {
                curv = 1 / r;
            }
            path.get(i).setCurvature(curv);
        }
    }


    /**
     * Each point on the path will have a target velocity the robot tries to reach.
     * The robot uses the target velocity of the point closest to it when calculating the target left and right wheel speeds.
     * When calculating the target velocity for a point we take into account the curvature at the point so the robot slows down around sharp turns.
     * (For this reason you must run the methods in order).
     * <p>
     * The last of the five methods used in the path generation, needed for the pure pursuit.
     * (see the Pure pursuit article, 'Path Generation' > 'Velocities' , Page 8)
     *
     * @param maxAcceleration rhe acceleration constant
     * @author paulo
     */
    public void generateVelocity(double maxAcceleration, double pathMaximumVelocity) {
        //Each point is given a speed based on its curvature, and the maximum velocity allowed.
        for (int i = 0; i < this.length(); i++) {
            if (this.getWaypoint(i).getCurvature() != 0) //prevent zero division error
                this.getWaypoint(i).setSpeed(Math.min(pathMaximumVelocity, Constants.K_CURVE / this.getWaypoint(i).getCurvature()));
            else
                this.getWaypoint(i).setSpeed(pathMaximumVelocity);
        }
        this.getWaypoint(-1).setSpeed(0);

        //Goes in reverse from the end to the beggining, lowering the speeds so that the robot doesn't de accelerate as fast.
        for (int i = this.length() - 2; i >= 0; i--) {
            getWaypoint(i).setSpeed(
                    Math.min(getWaypoint(i).getSpeed(), Math.sqrt(
                            Math.pow(getWaypoint(i + 1).getSpeed(), 2) + 2 * maxAcceleration * Waypoint.distance(getWaypoint(i), getWaypoint(i + 1))
                            )
                    )
            );
        }
    }


    // ----== Functions for Dubin's path generation: ==----


    public void createDubinCurve(Point start_position, double start_angle, Point end_position, double end_angle, double minimum_radius) {
        /*
        There are three stages to this algorithm.
        1. we create two pairs of circles tangent to each of the two points, and we choose which is the correct one
        2. find the correct tangent points
        3. create points on the circles
         */


    }

    /**
     * Find the tangent points
     *
     * @param c1
     * @param c2
     * @param min_radius
     * @return
     */
    private Point[][] generateDubinKeyPoints(Point c1, Point c2, double min_radius) {
        if (2 * min_radius > Point.distance(c1, c2)) return null; //return null if both circles intersect
        if (2 * min_radius == Point.distance(c1, c2))
            return new Point[][]{{Point.average(c1, c2)}}; //if both circles are tangent
        /* The goal of this method is to return the tangent points between both circles.
        To find crossing tangent lines (In LSR and RLS cases) we do this calculation:
        Create a circle where the diameter is the distance between both circles.
        Create a circle around one circles center with a radius twice the size of the radius.
        Find the intersection between both circles. (there are two)
        Calculate the vector between the intersection and the other circle.
        Create a point at the center between the circle and the intersection. (first point)
        Add the vector to the first point to get the second. (second point)
         */

        /*
        d=sqr((x1-x0)^2 + (y1-y0)^2)
        a=(r0^2-r1^2+d^2)/(2*d)
        h=sqr(r0^2-a^2)
        x2=x0+a*(x1-x0)/d
        y2=y0+a*(y1-y0)/d
        x3=x2+h*(y1-y0)/d       // also x3=x2-h*(y1-y0)/d
        y3=y2-h*(x1-x0)/d       // also y3=y2+h*(x1-x0)/d
         */
        Point circleDistance = Point.average(c1, c2);

        double d = Point.distance(c1, circleDistance); //distance between both intersecting circles
        double a = (4 * min_radius * min_radius - Math.pow(Point.distance(c1, c2) / 2, 2) + d * d) / (2 * d); //distance of c1 from the intersection line
        double h = Math.sqrt(4 * min_radius * min_radius - a * a); //distance of the line connecting both circles from the intersection.
        Vector v1 = new Vector(c1, c2); //create a vector from the first center to the second (which is the same as from the first one to the larger circle.
        Point p2 = v1.normalize().multiply(a).add(c1); //p2 is the point on the intersection between the centerline and the intersection line.
        v1.rotate(90); // rotates 90 counter clockwise
        Point intersect1 = v1.normalize().multiply(h).add(p2);
        Point intersect2 = v1.normalize().multiply(-h).add(p2);
        System.out.println(" " + a + " " + h);
        Vector v2 = new Vector(intersect1, c2);
        Point tan1 = Point.average(c1, intersect1); //the tangent counter clockwise of the center line
        Point tan2 = v2.add(tan1); //the tangent counter clockwise of the center line

        Vector v3 = new Vector(intersect2, c2);
        Point tan3 = Point.average(c1, intersect2); //the tangent clockwise of the center line
        Point tan4 = v3.add(tan3);

        Vector v4 = new Vector(c1, c2);
        v4.rotate(90); //rotate 90 counter clockwise
        Point tan5 = v4.normalize().multiply(min_radius).add(c1);
        Point tan6 = v4.normalize().multiply(min_radius).add(c2);
        Point tan7 = v4.normalize().multiply(-min_radius).add(c1);
        Point tan8 = v4.normalize().multiply(-min_radius).add(c2);

        // counterclock cross, clockwise cross, counter clockwise from c1, clockwise from c1
        return new Point[][]{{tan1, tan2}, {tan3, tan4}, {tan5, tan6}, {tan7, tan8}};
    }

    @Override
    public String toString() {
        return "Path{" + "path=" + path + '}';
    }
}
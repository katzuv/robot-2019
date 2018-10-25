package robot.subsystems.drivetrain.pure_pursuit;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.drivetrain.Drivetrain;

/**
 *
 */
public class PurePursue extends Command {
    private double lastLeftEncoder;
    private double lastRightEncoder;
    private double initAngle;
    private Path path;
    private Point robot;
    private Drivetrain drive;

    /**
     * @param path
     */
    public PurePursue(Path path) {
        drive = Robot.drivetrain;
        this.path = path;
        robot = new Point(0, 0);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        lastLeftEncoder = drive.getLeftDistance();
        lastRightEncoder = drive.getRightDistance();
        initAngle = Robot.navx.getAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        updatePoint();


    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

    private void updatePoint() {
        //change in (change left encoder value + change in right encoder value)/2
        double distance = ((drive.getLeftDistance() - lastLeftEncoder) + (drive.getRightDistance() - lastRightEncoder)) / 2;
        robot.setX(robot.getX() + distance * Math.cos(drive.getAngle() * (Math.PI / 180.0)));
        robot.setY(robot.getY() + distance * Math.sin(drive.getAngle() * (Math.PI / 180.0)));

        lastLeftEncoder = drive.getLeftDistance();
        lastRightEncoder = drive.getRightDistance();
    }

    //Only Part of the function! doesn't run function through all of the path
    //https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899

    /**
     * @param ref
     * @param lookahead
     * @param point1
     * @param point2
     * @return
     */
    private Point findLookaheadInSegment(Point ref, double lookahead, PathPoint point1, PathPoint point2) {
        Vector p = new Vector(point2, point1);
        Vector f = new Vector(point1, ref);

        //p*p + 2*f*p + f*f - r*r = 0
        double a = p.dot(p);
        double b = 2 * f.dot(p);
        double c = f.dot(f) - lookahead * lookahead;
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0)
            return null; //means that the circle doesnt reach the line
        else {
            discriminant = Math.sqrt(discriminant);
            double opt1 = (-b - discriminant) / (2 * a);
            double opt2 = (-b + discriminant) / (2 * a);
            if (opt1 >= 0 && opt1 <= 1) {
                return p.multiply(opt1).add(point1);
            }
            if (opt2 >= 0 && opt2 <= 1)
                return p.multiply(opt2).add(point1);
        }
        return null; //means that the segment is entirely inside the line.
    }

    /**
     * @return
     */
    private Point findLookaheadInPath() {
        return null;
    }

    /**
     * @return
     */
    private Point closestPoint() {
        return null;
    }

    /**
     * @return
     */
    private double curvatureCalculate() {
        return 0;
    }

    /**
     * @return
     */
    public double getRightSpeed() {
        return 0;
    }

    /**
     * @return
     */
    public double getLeftSpeed() {
        return 0;
    }

}
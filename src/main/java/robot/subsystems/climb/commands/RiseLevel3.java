package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.climb.Constants;
import robot.subsystems.climb.TiltUtils;

import java.awt.geom.Point2D;

import static robot.Robot.climb;

/**
 *
 */
public class RiseLevel3 extends Command {
    private double targetHeight = Constants.LEVEL_THREE_LEG_LENGTH;
    private Point2D armDimensions;
    //gamers, lose yourself and rise up
    public RiseLevel3() {
        requires(climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        climb.setLegFLHeight(targetHeight, 0);
        climb.setLegFRHeight(targetHeight, 0);
        climb.setLegBLHeight(targetHeight, 0);
        climb.setLegBRHeight(targetHeight, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() { //TODO: either try out an average, a timeout or an angle consideration aswell
        return climb.getLegFLHeight()>targetHeight-Constants.CLIMB_TOLERANCE &&
                climb.getLegFRHeight()>targetHeight-Constants.CLIMB_TOLERANCE &&
                climb.getLegBLHeight()>targetHeight-Constants.CLIMB_TOLERANCE &&
                climb.getLegBRHeight()>targetHeight-Constants.CLIMB_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
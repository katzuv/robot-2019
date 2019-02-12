package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.climb.Constants;
import robot.subsystems.climb.TiltUtils;

import static robot.Robot.climb;

/**
 * An FRC command, climbs to the third level of the HAB zone.
 * The motors are run at the execute, and using arbitrary feedforward, tipping is prevented.
 * This class only move upwards, the rest is done in separate commands.
 *
 * @author paulo
 */
public class RiseLevel3 extends Command {
    private double targetHeight = Constants.LEVEL_THREE_LEG_LENGTH;

    //gamers, (lose yourself and) rise up
    public RiseLevel3() {
        requires(climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        climb.setLegFLHeight(targetHeight, TiltUtils.getLegLength(-Constants.LEG_X_DIMENSION, Constants.LEG_Y_DIMENSION, Robot.navx.getPitch(), Robot.navx.getRoll()));
        climb.setLegFRHeight(targetHeight, TiltUtils.getLegLength(Constants.LEG_X_DIMENSION, Constants.LEG_Y_DIMENSION, Robot.navx.getPitch(), Robot.navx.getRoll()));
        climb.setLegBLHeight(targetHeight, TiltUtils.getLegLength(-Constants.LEG_X_DIMENSION, -Constants.LEG_Y_DIMENSION, Robot.navx.getPitch(), Robot.navx.getRoll()));
        climb.setLegBRHeight(targetHeight, TiltUtils.getLegLength(Constants.LEG_X_DIMENSION, -Constants.LEG_Y_DIMENSION, Robot.navx.getPitch(), Robot.navx.getRoll()));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() { //TODO: either try out an average, a timeout or an angle consideration aswell
        return climb.getLegFLHeight() > targetHeight - Constants.CLIMB_TOLERANCE &&
                climb.getLegFRHeight() > targetHeight - Constants.CLIMB_TOLERANCE &&
                climb.getLegBLHeight() > targetHeight - Constants.CLIMB_TOLERANCE &&
                climb.getLegBRHeight() > targetHeight - Constants.CLIMB_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
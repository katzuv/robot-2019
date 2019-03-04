package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.climb.Climb;
import robot.subsystems.climb.Constants;
import robot.subsystems.climb.TiltUtils;

import static robot.Robot.climb;

/**
 *
 */
public class RiseToHeightCombined extends Command {
    private double targetHeight = Constants.LEVEL_THREE_LEG_LENGTH;

    //gamers, (lose yourself and) rise up
    public RiseToHeightCombined(double targetHeight) {
        this.targetHeight = targetHeight;
        requires(climb);
    }

    public RiseToHeightCombined(Climb.HAB_LEG_HEIGHTS height_state) {
        this(height_state.getHABHHeight());
    }


    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        double minimumLeg = Math.min(climb.getLegBLHeight(), Math.min(climb.getLegBRHeight(), Math.min(climb.getLegFLHeight(), climb.getLegFRHeight())));
        climb.setLegFLHeight(targetHeight, minimumLeg - climb.getLegFLHeight(), TiltUtils.getLegLength(-Constants.LEG_X_DIMENSION, Constants.LEG_Y_DIMENSION, Robot.navx.getPitch(), Robot.navx.getRoll()));
        climb.setLegFRHeight(targetHeight, minimumLeg - climb.getLegFRHeight(), TiltUtils.getLegLength(Constants.LEG_X_DIMENSION, Constants.LEG_Y_DIMENSION, Robot.navx.getPitch(), Robot.navx.getRoll()));
        climb.setLegBLHeight(targetHeight, minimumLeg - climb.getLegBLHeight(), TiltUtils.getLegLength(-Constants.BACK_LEG_X_DIMENSION, -Constants.BACK_LEG_Y_DIMENSION, Robot.navx.getPitch(), Robot.navx.getRoll()));
        climb.setLegBRHeight(targetHeight, minimumLeg - climb.getLegBRHeight(), TiltUtils.getLegLength(Constants.BACK_LEG_X_DIMENSION, -Constants.BACK_LEG_Y_DIMENSION, Robot.navx.getPitch(), Robot.navx.getRoll()));

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(climb.getLegFLHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegFRHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegBLHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegBRHeight() - targetHeight) < Constants.CLIMB_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
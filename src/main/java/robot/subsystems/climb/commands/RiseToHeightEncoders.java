package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.climb.Climb;
import robot.subsystems.climb.Constants;

import static robot.Robot.climb;

/**
 * An FRC command, climbs to the third level of the HAB zone.
 * The motors are run at the execute, and using arbitrary feedforward, tipping is prevented.
 * This class only move upwards, the rest is done in separate commands.
 *
 * @author paulo
 */
public class RiseToHeightEncoders extends Command {
    private double targetHeight = Constants.LEVEL_THREE_LEG_LENGTH;

    //gamers, (lose yourself and) rise up
    public RiseToHeightEncoders(double targetHeight) {
        this.targetHeight = targetHeight;
        requires(climb);
    }

    public RiseToHeightEncoders(Climb.HAB_LEG_HEIGHTS height_state) {
        this(height_state.getHABHHeight());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        climb.setLegDriveHeight(targetHeight, 0); //TODO: should be get drive height
        climb.setLegBLHeight(targetHeight-Constants.BACK_LEFT_STARTING_OFFSET, 0);
        climb.setLegBRHeight(targetHeight-Constants.BACK_RIGHT_STARTING_OFFSET, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() { //TODO: either try out an average, a timeout or an angle consideration aswell
        return Math.abs(climb.getLegFLHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegFRHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegBLHeight() - targetHeight) < Constants.CLIMB_TOLERANCE &&
                Math.abs(climb.getLegBRHeight() - targetHeight) < Constants.CLIMB_TOLERANCE;
    }

    // Called once after isFinished returns true
    protected void end() {
        climb.setLegBLSpeed(0);
        climb.setLegBRSpeed(0);
        climb.setLegDriveSpeed(0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
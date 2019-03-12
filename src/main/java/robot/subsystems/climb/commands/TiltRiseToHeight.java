package robot.subsystems.climb.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.climb.Climb;

/**
 * Essentially the same as RiseToHeight, but makes sure the back legs start more out, to prevent tipping.
 */
public class TiltRiseToHeight extends CommandGroup {

    public TiltRiseToHeight(double targetHeight) {
        addSequential(new MoveBackLegs(0.0125));
        addSequential(new RiseToHeightEncoders(targetHeight));
    }

    public TiltRiseToHeight(Climb.HAB_LEG_HEIGHTS height_state){
        this(height_state.getHABHHeight());
    }
}
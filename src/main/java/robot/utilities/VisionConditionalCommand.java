package robot.utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import robot.Robot;

/**
 * Checks whether the robot sees a vision target
 */
public class VisionConditionalCommand extends ConditionalCommand {
    private NetworkTableEntry seenEntry = Robot.visionTable.getEntry("tape_seen");

    public VisionConditionalCommand(Command onTrue) {
        super(onTrue);
    }

    @Override
    protected boolean condition() {
        return seenEntry.getBoolean(false);
    }


}
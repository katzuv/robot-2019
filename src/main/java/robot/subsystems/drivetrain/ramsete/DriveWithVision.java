package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.elevator.Constants;
import robot.subsystems.commandGroups.HatchScoring;

/**
 *
 */
public class DriveWithVision extends CommandGroup {

    public DriveWithVision() {
//        addSequential(new DrivePathNew(trajectory));
        addSequential(new VisionTarget());
        addSequential(new HatchScoring(Constants.ELEVATOR_STATES.SHIP_HATCH,true));
    }
}

package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;

/**
 *
 */
public class VisionPlaceHatch extends CommandGroup {

    public VisionPlaceHatch(Constants.ELEVATOR_HEIGHTS height) {
        addParallel(new ElevatorCommand(height));
        addSequential(new VisionDrive());
//        addSequential(new HatchScoring(height, false));
    }
}
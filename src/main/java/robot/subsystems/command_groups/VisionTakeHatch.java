package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Flower;

/**
 *
 */
public class VisionTakeHatch extends CommandGroup {

    public VisionTakeHatch() {
        addParallel(new ElevatorCommand(Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH));
        addSequential(new VisionDrive());
        addSequential(new Flower(true));
    }
}
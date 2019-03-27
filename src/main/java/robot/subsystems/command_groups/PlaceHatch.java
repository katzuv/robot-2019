package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.ExtensionPlate;
import robot.subsystems.hatch_intake.commands.Flower;

/**
 *
 */
public class PlaceHatch extends CommandGroup {

    public PlaceHatch(Constants.ELEVATOR_STATES height) {
        addSequential(new ElevatorCommand(height));
        addSequential(new WaitCommand(0.1));
        addSequential(new Flower(true));// release hatch;

        addSequential(new WaitCommand(0.02));
        addSequential(new ExtensionPlate(true));//extend
        addSequential(new WaitCommand(0.5));
        addSequential(new ElevatorCommand(height.getLevelHeight() - 0.06));
    }
}
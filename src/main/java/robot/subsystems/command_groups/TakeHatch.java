package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.hatch_intake.commands.ExtensionPlate;

public class TakeHatch extends CommandGroup {

    public TakeHatch() {

        addSequential(new ElevatorCommand(Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH));
        addSequential(new Flower(true)); //open flower
        addSequential(new ExtensionPlate(true));//extend
        addSequential(new WaitCommand(0.7));
        addSequential(new Flower(false));// take hatch;
        addSequential(new WaitCommand(0.3));
        //return to previous form
        addSequential(new ExtensionPlate(false));
    }
}

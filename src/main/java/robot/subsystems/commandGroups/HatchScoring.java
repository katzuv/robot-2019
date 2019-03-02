package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Gripper;
import robot.subsystems.hatch_intake.commands.GripperTransportation;

/**
 *This command group gets a level as parameter and then rise to this level,
 * at then place the hatch
 * @author Orel
 */

public class HatchScoring extends CommandGroup {

    public HatchScoring(Constants.ELEVATOR_STATES height) {

        addSequential(new ElevatorCommand(height));
        addSequential(new GripperTransportation(true));//extend
        addSequential(new Gripper(true));// release hatch;
        addSequential(new WaitCommand(0.5));
        addSequential(new ElevatorCommand(height.getLevelHeight() - 0.07));
        addSequential(new WaitCommand(0.3));
        //return to previous form
        addSequential(new GripperTransportation(false));
        addSequential(new WaitCommand(0.2));
        addSequential(new Gripper(false));
//        addSequential(new ElevatorCommand(0));

    }
}

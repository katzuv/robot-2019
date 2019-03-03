package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;

public class PlaceHatch extends CommandGroup {

    public PlaceHatch(Constants.ELEVATOR_STATES height) {

        addSequential(new ElevatorCommand(height));

        addSequential(new GripperTransportation(true));//extend
        addSequential(new Gripper(true));// release hatch;
        addSequential(new WaitCommand(0.5));
        addSequential(new ElevatorCommand(height.getLevelHeight() - 0.10));
        addSequential(new WaitCommand(0.3));
        //return to previous form
        addSequential(new GripperTransportation(false));
        addSequential(new WaitCommand(0.2));
        addSequential(new Gripper(false));
        addSequential(new ElevatorCommand(0));

    }
}

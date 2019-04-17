package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;

public class TakeHatch extends CommandGroup {

    public TakeHatch() {

        addSequential(new ElevatorCommand(Constants.ELEVATOR_STATES.LEVEL1_HATCH));
        addSequential(new Gripper(true)); //open flower
        addSequential(new GripperTransportation(true));//extend
        addSequential(new WaitCommand(0.7));
        addSequential(new Gripper(false));// take hatch;
        addSequential(new WaitCommand(0.3));
        //return to previous form
        addSequential(new GripperTransportation(false));
    }
}

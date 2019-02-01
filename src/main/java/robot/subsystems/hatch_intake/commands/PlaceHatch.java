package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PlaceHatch extends CommandGroup {
    public PlaceHatch() {
        addSequential(new GripperTransportation(true));//extend
        addSequential(new Gripper(true));// release hatch
        //return to previous form
        addSequential(new GripperTransportation(false));
        addSequential(new Gripper(false));
    }
}

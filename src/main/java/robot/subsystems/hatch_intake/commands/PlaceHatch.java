package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PlaceHatch extends CommandGroup {
    public PlaceHatch() {
        addSequential(new GripperMovement(true));//extend
        addSequential(new Gripper(true));// release hatch
        //return to previous form
        addSequential(new GripperMovement(false));
        addSequential(new Gripper(false));
    }
}

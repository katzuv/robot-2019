package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PlaceHatch extends CommandGroup {
    public PlaceHatch() {
        addSequential(new FlowerTransportation(true));//extend
        addSequential(new Grab(true));// release hatch
        //return to previous form
        addSequential(new FlowerTransportation(false));
        addSequential(new Grab(false));
    }
}

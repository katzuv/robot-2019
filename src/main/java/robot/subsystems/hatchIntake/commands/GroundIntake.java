package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GroundIntake extends CommandGroup {

    public GroundIntake() {
        addSequential(new HatchTransportation());//lift hatch if inside
        addSequential(new Grab(false));//put it on the flower
    }
}

package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GroundIntake extends CommandGroup {
    /*
    a command group used to both lift the hatch and place it on the flower
     */

    public GroundIntake() {
        addSequential(new HatchTransportation());//lift hatch if inside
        addSequential(new Gripper(false));//put it on the flower
    }
}

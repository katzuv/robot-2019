package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GroundIntake extends CommandGroup {

    public GroundIntake() {
            addSequential(new HatchTransportation());//lift hatch if inside
            addSequential(new Gripper(false));//put it on the flower
    }
}

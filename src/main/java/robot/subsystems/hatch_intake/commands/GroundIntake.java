package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GroundIntake extends CommandGroup {

    public GroundIntake() {//TODO: this command isn't right. we need to fix it in the future. theres no testing whether the hatch is inside, and no safety features.
            addSequential(new HatchTransportation(false));//lift hatch if inside
            addSequential(new Gripper(false));//put it on the flower
    }
}

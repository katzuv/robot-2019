package robot.subsystems.hatchIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;

public class GroundIntake extends CommandGroup {

    public GroundIntake() {
        if (!Robot.GROUNDINTAKE.HaveGamePiece()) {
            addSequential(new HatchTransportation());//lift hatch if inside
            addSequential(new Gripper(false));//put it on the flower
        }
    }
}

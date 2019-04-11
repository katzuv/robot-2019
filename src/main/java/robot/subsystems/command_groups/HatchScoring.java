package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.elevator.Constants;

public class HatchScoring extends CommandGroup {
    

    public HatchScoring(Constants.ELEVATOR_STATES height, boolean retract) {
        addSequential(new PlaceHatch(height));

        if (retract) {
            addSequential(new WaitCommand(0.3));
            addSequential(new RetractHatch());
        }
    }

    public HatchScoring(Constants.ELEVATOR_STATES height) {
        addSequential(new PlaceHatch(height));
        addSequential(new WaitCommand(0.3));
        addSequential(new RetractHatch());
    }
}

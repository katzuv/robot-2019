package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import robot.Robot;

public class ScoringChooser extends ConditionalCommand {

    public ScoringChooser(Command cargo, Command hatch) {
        super(cargo, hatch);
    }

    protected boolean condition() {
        return Robot.cargoIntake.isCargoInside();
    }
}
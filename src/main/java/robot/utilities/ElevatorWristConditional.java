package robot.utilities;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import robot.Robot;

public class ElevatorWristConditional extends ConditionalCommand {

    public ElevatorWristConditional(Command onTrue) {
        super(onTrue);
    }

    @Override
    protected boolean condition() {
        return Robot.elevator.getHeight() > 0.08;
    }

}

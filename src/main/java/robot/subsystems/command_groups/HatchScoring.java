package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Fangs;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.wrist_control.commands.WristTurn;

/**
 * Hatch scoring command group, new to the wrist hatch mechanism.
 */
public class HatchScoring extends CommandGroup {

    public HatchScoring(double height) {
        addSequential(
                new CommandGroup() {
                    {
                        addParallel(new ElevatorCommand(height));
                        addParallel(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));
                    }
                }
        );
        addSequential(new Flower(true));
        addSequential(new WaitCommand(0.2));
        addSequential(new Fangs(true, 0.5));
    }

    public HatchScoring(Constants.ELEVATOR_HEIGHTS height) {
        this(height.getLevelHeight());
    }
}
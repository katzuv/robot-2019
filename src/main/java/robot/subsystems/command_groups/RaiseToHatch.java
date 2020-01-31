package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.wrist_control.commands.WristTurn;
import robot.utilities.SetRocket;

/**
 *
 */
public class RaiseToHatch extends SequentialCommandGroup {

    public RaiseToHatch(Constants.ELEVATOR_HEIGHTS height) {
        addCommands((new SetRocket(true)),
                (new ParallelCommandGroup() {
                    {
                        addCommands((new ElevatorCommand(height)),
                                (new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD)));
                    }
                }));
    }
}
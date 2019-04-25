package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.wrist_control.commands.WristTurn;
import robot.utilities.SetRocket;

/**
 *
 */
public class RaiseToHatch extends CommandGroup {

    public RaiseToHatch(Constants.ELEVATOR_HEIGHTS height) {
        addSequential(new SetRocket(true));
        addSequential(new CommandGroup() {
            {
                addParallel(new ElevatorCommand(height));
                if (height == Constants.ELEVATOR_HEIGHTS.LEVEL3_HATCH_VISION) {
                    addParallel(new WristTurn(118));
                } else {
                    addParallel(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));
                }
            }
        });
    }
}
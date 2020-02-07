package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import robot.subsystems.elevator.Elevator;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.wrist_control.Constants;
import robot.subsystems.wrist_control.WristControl;

/**
 *
 */
public class WristAndElevatorCommand extends ParallelCommandGroup {

    public WristAndElevatorCommand(Elevator elevator, WristControl wristControl, double angle, double height) {
        addCommands(new ElevatorCommand(elevator, height),
                (new WristTurn(wristControl, angle, 1.5)));
    }

    public WristAndElevatorCommand(Elevator elevator, WristControl wristControl, Constants.WRIST_ANGLES wristAngles, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS elevatorHeight) {
        this(elevator, wristControl, wristAngles.getValue(), elevatorHeight.getLevelHeight());
    }
}
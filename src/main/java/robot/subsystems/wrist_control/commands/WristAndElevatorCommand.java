package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.wrist_control.Constants;

import static robot.Robot.elevator;
import static robot.Robot.wristControl;

/**
 *
 */
public class WristAndElevatorCommand extends ParallelCommandGroup {

    public WristAndElevatorCommand(double angle, double height) {
        addCommands(new ElevatorCommand(height),
                (new WristTurn(angle,1.5)));
    }

    public WristAndElevatorCommand(Constants.WRIST_ANGLES wristAngles, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS elevatorHeight){
        this(wristAngles.getValue(), elevatorHeight.getLevelHeight());
    }
}
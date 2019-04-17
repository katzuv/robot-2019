package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.wrist_control.Constants;

import static robot.Robot.elevator;
import static robot.Robot.wristControl;

/**
 *
 */
public class WristAndElevatorCommand extends CommandGroup {

    public WristAndElevatorCommand(double angle, double height) {
        addParallel(new ElevatorCommand(height));
        addSequential(new WristTurn(angle,1.5));
    }

    public WristAndElevatorCommand(Constants.WRIST_ANGLES wristAngles, robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS elevatorHeight){
        this(wristAngles.getValue(), elevatorHeight.getLevelHeight());
    }
}
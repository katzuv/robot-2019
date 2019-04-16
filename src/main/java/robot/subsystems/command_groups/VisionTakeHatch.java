package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.wrist_control.commands.WristTurn;
import robot.utilities.SetRocket;

/**
 *
 */
public class VisionTakeHatch extends CommandGroup {

    public VisionTakeHatch() {
        addSequential(new SetRocket(true));
        addSequential(new CommandGroup() {
            {
                addParallel(new ElevatorCommand(Constants.ELEVATOR_HEIGHTS.LOADING_STATION));
                addParallel(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));
            }
        });
        addSequential(new Flower(true));
        addSequential(new WaitCommand(0.2));
        addSequential(new VisionDrive(robot.subsystems.drivetrain.Constants.HATCH_TARGET_DISTANCE));
        addSequential(new WaitCommand(0.2));
        addSequential(new Flower(false));

    }
}
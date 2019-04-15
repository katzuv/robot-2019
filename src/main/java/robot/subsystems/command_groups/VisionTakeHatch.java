package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.wrist_control.commands.WristTurn;

/**
 *
 */
public class VisionTakeHatch extends CommandGroup {

    public VisionTakeHatch() {
        addSequential(new CommandGroup() {
            {
                addParallel(new ElevatorCommand(Constants.ELEVATOR_HEIGHTS.LOADING_STATION));
                addParallel(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));
            }
        });
        addSequential(new Flower(true));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));
        addSequential(new Flower(false));

    }
}
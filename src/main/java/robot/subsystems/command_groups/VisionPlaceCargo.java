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
public class VisionPlaceCargo extends CommandGroup {

    public VisionPlaceCargo(int state) {
        addSequential(new SetRocket(false));
        addSequential(new VisionDrive(robot.subsystems.drivetrain.Constants.CARGO_TARGET_DISTANCE));
        new WaitCommand(0.2);
        addSequential(new CargoScoring(state, false));
        addSequential(new CargoRubbing(false));
    }
}
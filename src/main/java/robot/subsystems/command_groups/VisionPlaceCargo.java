package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.wrist_control.Constants;
import robot.subsystems.wrist_control.commands.GripperControl;
import robot.subsystems.wrist_control.commands.WristTurn;
import robot.utilities.SetRocket;

/**
 *
 */
public class VisionPlaceCargo extends CommandGroup {

    public VisionPlaceCargo(int state) {
        if(state == 0) {
            addSequential(new SetRocket(true));
            addSequential(new CargoScoring(state, false));
            addSequential(new WaitCommand(0.2));
            addSequential(new VisionDrive(1));
            addSequential(new GripperControl(Constants.GRIPPER_SPEED.SHIP.getValue()), 0.3);
        }
        else {
            addSequential(new SetRocket(false));
            addSequential(new VisionDrive(robot.subsystems.drivetrain.Constants.CARGO_TARGET_DISTANCE));
            addSequential(new CargoScoring(state, false));
            addSequential(new WaitCommand(0.2));
            addSequential(new CargoRubbing(false));
        }

    }
}
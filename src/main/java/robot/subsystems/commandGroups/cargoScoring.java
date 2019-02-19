package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.cargo_intake.Constants;
import robot.subsystems.cargo_intake.commands.GripperControl;
import robot.subsystems.cargo_intake.commands.WristTurn;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Gripper;

/**
 *
 */
public class cargoScoring extends CommandGroup {

    public cargoScoring(robot.subsystems.elevator.Constants.ELEVATOR_STATES state) {
        addParallel(new ElevatorCommand(state.getLevelHeight()));
        addSequential(new WristTurn(Constants.WRIST_ANGLES.SHOOTING));
        addSequential(new GripperControl(Constants.GRIPPER_SHOOT_SPEED));
        addSequential(new WristTurn(Constants.WRIST_ANGLES.INITIAL));

        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
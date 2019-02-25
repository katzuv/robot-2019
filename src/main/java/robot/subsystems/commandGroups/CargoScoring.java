package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.cargo_intake.Constants;
import robot.subsystems.cargo_intake.commands.GripperControl;
import robot.subsystems.cargo_intake.commands.WristTurn;
import robot.subsystems.elevator.commands.ElevatorCommand;


/**
 *
 */
public class CargoScoring extends CommandGroup {

    public CargoScoring(robot.subsystems.elevator.Constants.ELEVATOR_STATES height, Constants.WRIST_ANGLES angle) {
        addParallel(new WristTurn(angle));
        addSequential(new ElevatorCommand(height));
        addSequential(new WaitCommand(1));
        addSequential(new GripperControl(Constants.GRIPPER_SHOOT_SPEED));
        addSequential(new WaitCommand(0.4));
        addParallel(new WristTurn(Constants.WRIST_ANGLES.INITIAL));
        addSequential(new ElevatorCommand(0));

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
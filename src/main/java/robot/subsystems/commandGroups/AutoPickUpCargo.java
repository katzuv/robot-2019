package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.cargo_intake.Constants;
import robot.subsystems.cargo_intake.commands.GripperControl;
import robot.subsystems.cargo_intake.commands.WristTurn;
import robot.subsystems.drivetrain.commands.GamePiecePickup;

/**
 *
 */
public class AutoPickUpCargo extends CommandGroup {

    public AutoPickUpCargo() {
        //addSequential(new GamePiecePickup(robot.subsystems.drivetrain.Constants.DRIVING_TARGETS.CARGO));
        //addSequential(new WristTurn(Constants.WRIST_ANGLES.INTAKE));
        //addParallel(new GripperControl(Constants.GRIPPER_SPEED.INTAKE));
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
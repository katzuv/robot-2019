package robot.subsystems.cargo_intake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.cargo_intake.Constants;

/**
 * This cargo intake command group follows three basic steps, put the wrist up against the cargo,
 * pull the cargo in the system and lift the wrist back up
 */
public class IntakeCargoAndFoldWrist extends CommandGroup {

    public IntakeCargoAndFoldWrist() {
        addSequential(new WristTurn(Constants.INITIAL_ANGLE));
        addSequential(new GripperControl(0.75, false));
        addSequential(new WristTurn(90));
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
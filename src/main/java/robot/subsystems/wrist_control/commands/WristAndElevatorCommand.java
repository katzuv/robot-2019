package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.elevator.commands.ElevatorCommand;

import static robot.Robot.elevator;
import static robot.Robot.wristControl;

/**
 *
 */
public class WristAndElevatorCommand extends CommandGroup {

    public WristAndElevatorCommand(double angle, double height) {
        addSequential(new ElevatorCommand(height));
        addSequential(new WristTurn(angle));
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
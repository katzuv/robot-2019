package robot.subsystems.cargoIntake.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class CargoIntake extends CommandGroup {

    public CargoIntake() {
        WristTurn wristTurn = new WristTurn(0);
        GripperControl gripper = new GripperControl(0.75, false);
        WristTurn wristTurn = new WristTurn(90);
        addSequential(gripper);
        addSequential(wristTurn);
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
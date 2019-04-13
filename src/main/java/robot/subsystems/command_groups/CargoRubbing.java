package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.wrist_control.commands.GripperControl;
import robot.subsystems.wrist_control.commands.WristTurn;

/**
 *
 */
public class CargoRubbing extends CommandGroup {

    public CargoRubbing(int state, boolean isBackward) {

        Constants.ELEVATOR_HEIGHTS height = getHeight(state, isBackward);
        robot.subsystems.wrist_control.Constants.WRIST_ANGLES angle = getAngle(state, isBackward);
        robot.subsystems.wrist_control.Constants.GRIPPER_SPEED speed = getSpeed(state, isBackward);

        addParallel(new WristTurn(angle));
        addSequential(new ElevatorCommand(height));
        addParallel(new GripperControl(speed));
        addParallel(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));

    }

    public Constants.ELEVATOR_HEIGHTS getHeight(int state, boolean isBackward) {

        if (isBackward) {

            switch (state) {
                case 0:
                    return Constants.ELEVATOR_HEIGHTS.SHIP_CARGO_BACKWARD;
                case 1:
                    return Constants.ELEVATOR_HEIGHTS.LEVEL1_CARGO_BACKWARD;
                case 2:
                    return Constants.ELEVATOR_HEIGHTS.LEVEL2_CARGO_BACKWARD;
                case 3:
                    return Constants.ELEVATOR_HEIGHTS.LEVEL3_CARGO_BACKWARD;
            }

        } else {
            switch (state) {
                case 0:
                    return Constants.ELEVATOR_HEIGHTS.SHIP_CARGO;
                case 1:
                    return Constants.ELEVATOR_HEIGHTS.LEVEL1_CARGO;
                case 2:
                    return Constants.ELEVATOR_HEIGHTS.LEVEL2_CARGO;
                case 3:
                    return Constants.ELEVATOR_HEIGHTS.LEVEL3_CARGO;

            }

        }
        return null;
    }


    public robot.subsystems.wrist_control.Constants.GRIPPER_SPEED getSpeed(int state, boolean isBackward) {

        if (isBackward) {

            switch (state) {
                case 0:
                    return robot.subsystems.wrist_control.Constants.GRIPPER_SPEED.SHIP_BACKWARD;
                case 1:
                    return robot.subsystems.wrist_control.Constants.GRIPPER_SPEED.LEVEL_1_BACKWARD;
                case 2:
                    return robot.subsystems.wrist_control.Constants.GRIPPER_SPEED.LEVEL_2_BACKWARD;
                case 3:
                    return robot.subsystems.wrist_control.Constants.GRIPPER_SPEED.LEVEL_3_BACKWARD;
            }

        } else {
            switch (state) {
                case 0:
                    return robot.subsystems.wrist_control.Constants.GRIPPER_SPEED.SHIP;
                case 1:
                    return robot.subsystems.wrist_control.Constants.GRIPPER_SPEED.LEVEL_1;
                case 2:
                    return robot.subsystems.wrist_control.Constants.GRIPPER_SPEED.LEVEL_2;
                case 3:
                    return robot.subsystems.wrist_control.Constants.GRIPPER_SPEED.LEVEL_3;

            }

        }
        return null;
    }

    public robot.subsystems.wrist_control.Constants.WRIST_ANGLES getAngle(int state, boolean isBackward) {

        if (isBackward) {

            switch (state) {
                case 0:
                    return robot.subsystems.wrist_control.Constants.WRIST_ANGLES.SHIP_BACKWARD;
                case 1:
                    return robot.subsystems.wrist_control.Constants.WRIST_ANGLES.LEVEL_1_BACKWARD;
                case 2:
                    return robot.subsystems.wrist_control.Constants.WRIST_ANGLES.LEVEL_2_BACKWARD;
                case 3:
                    return robot.subsystems.wrist_control.Constants.WRIST_ANGLES.LEVEL_3_BACKWARD;
            }

        } else {
            switch (state) {
                case 0:
                    return robot.subsystems.wrist_control.Constants.WRIST_ANGLES.SHIP;
                case 1:
                    return robot.subsystems.wrist_control.Constants.WRIST_ANGLES.LEVEL_1;
                case 2:
                    return robot.subsystems.wrist_control.Constants.WRIST_ANGLES.LEVEL_2;
                case 3:
                    return robot.subsystems.wrist_control.Constants.WRIST_ANGLES.LEVEL_3;

            }

        }
        return null;
    }
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

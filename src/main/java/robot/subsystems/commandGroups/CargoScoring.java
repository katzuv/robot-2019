package robot.subsystems.commandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.cargo_intake.Constants;
import robot.subsystems.cargo_intake.commands.GripperControl;
import robot.subsystems.cargo_intake.commands.WristTurn;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Gripper;

import static robot.subsystems.elevator.Constants.ELEVATOR_STATES;
import static robot.subsystems.cargo_intake.Constants.WRIST_ANGLES;
import static robot.subsystems.cargo_intake.Constants.GRIPPER_SPEED;


/**
 *
 */
public class CargoScoring extends CommandGroup {

    public CargoScoring(int state, boolean isBackward) {

        ELEVATOR_STATES height = getHeight(state, isBackward);
        WRIST_ANGLES angle = getAngle(state, isBackward);
        GRIPPER_SPEED speed = getSpeed(state, isBackward);

        addSequential(new CommandGroup(){
            {
                addParallel(new WristTurn(angle));
                addSequential(new ElevatorCommand(height));
            }
        });
        addSequential(new WaitCommand(0.25));
        addSequential(new GripperControl(speed));
        addSequential(new WaitCommand(0.4));
        addParallel(new WristTurn(WRIST_ANGLES.INITIAL));
        addSequential(new ElevatorCommand(0));
    }


    public ELEVATOR_STATES getHeight(int state, boolean isBackward) {

        if (isBackward) {

            switch (state) {
                case 0:
                    return ELEVATOR_STATES.SHIP_CARGO_BACKWARD;
                case 1:
                    return ELEVATOR_STATES.LEVEL1_CARGO_BACKWARD;
                case 2:
                    return ELEVATOR_STATES.LEVEL2_CARGO_BACKWARD;
                case 3:
                    return ELEVATOR_STATES.LEVEL3_CARGO_BACKWARD;
            }

        } else {
            switch (state) {
                case 0:
                    return ELEVATOR_STATES.SHIP_CARGO;
                case 1:
                    return ELEVATOR_STATES.LEVEL1_CARGO;
                case 2:
                    return ELEVATOR_STATES.LEVEL2_CARGO;
                case 3:
                    return ELEVATOR_STATES.LEVEL3_CARGO;

            }

        }
        return null;
    }


    public GRIPPER_SPEED getSpeed(int state, boolean isBackward) {

        if (isBackward) {

            switch (state) {
                case 0:
                    return GRIPPER_SPEED.SHIP_BACKWARD;
                case 1:
                    return GRIPPER_SPEED.LEVEL_1_BACKWARD;
                case 2:
                    return GRIPPER_SPEED.LEVEL_2_BACKWARD;
                case 3:
                    return GRIPPER_SPEED.LEVEL_3_BACKWARD;
            }

        } else {
            switch (state) {
                case 0:
                    return GRIPPER_SPEED.SHIP;
                case 1:
                    return GRIPPER_SPEED.LEVEL_1;
                case 2:
                    return GRIPPER_SPEED.LEVEL_2;
                case 3:
                    return GRIPPER_SPEED.LEVEL_3;

            }

        }
        return null;
    }

    public WRIST_ANGLES getAngle(int state, boolean isBackward) {

        if (isBackward) {

            switch (state) {
                case 0:
                    return WRIST_ANGLES.SHIP_BACKWARD;
                case 1:
                    return WRIST_ANGLES.LEVEL_1_BACKWARD;
                case 2:
                    return WRIST_ANGLES.LEVEL_2_BACKWARD;
                case 3:
                    return WRIST_ANGLES.LEVEL_3_BACKWARD;
            }

        } else {
            switch (state) {
                case 0:
                    return WRIST_ANGLES.SHIP;
                case 1:
                    return WRIST_ANGLES.LEVEL_1;
                case 2:
                    return WRIST_ANGLES.LEVEL_2;
                case 3:
                    return WRIST_ANGLES.LEVEL_3;

            }

        }
        return null;
    }
}



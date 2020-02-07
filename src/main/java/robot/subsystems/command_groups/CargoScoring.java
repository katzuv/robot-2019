package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import robot.subsystems.elevator.Elevator;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.wrist_control.WristControl;
import robot.subsystems.wrist_control.commands.WristTurn;

import static robot.subsystems.elevator.Constants.ELEVATOR_HEIGHTS;
import static robot.subsystems.wrist_control.Constants.GRIPPER_SPEED;
import static robot.subsystems.wrist_control.Constants.WRIST_ANGLES;


/**
 *
 */
public class CargoScoring extends ParallelCommandGroup {

    public CargoScoring(WristControl wristControl, Elevator elevator, int state, boolean isBackward) {

        ELEVATOR_HEIGHTS height = getHeight(state, isBackward);
        WRIST_ANGLES angle = getAngle(state, isBackward);
        GRIPPER_SPEED speed = getSpeed(state, isBackward);
        addCommands(new WristTurn(wristControl, angle), new ElevatorCommand(elevator, height));
    }


    public ELEVATOR_HEIGHTS getHeight(int state, boolean isBackward) {

        if (isBackward) {

            switch (state) {
                case 0:
                    return ELEVATOR_HEIGHTS.SHIP_CARGO_BACKWARD;
                case 1:
                    return ELEVATOR_HEIGHTS.LEVEL1_CARGO_BACKWARD;
                case 2:
                    return ELEVATOR_HEIGHTS.LEVEL2_CARGO_BACKWARD;
                case 3:
                    return ELEVATOR_HEIGHTS.LEVEL3_CARGO_BACKWARD;
            }

        } else {
            switch (state) {
                case 0:
                    return ELEVATOR_HEIGHTS.SHIP_CARGO;
                case 1:
                    return ELEVATOR_HEIGHTS.LEVEL1_CARGO;
                case 2:
                    return ELEVATOR_HEIGHTS.LEVEL2_CARGO;
                case 3:
                    return ELEVATOR_HEIGHTS.LEVEL3_CARGO;

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
                    return WRIST_ANGLES.FORWARD;
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



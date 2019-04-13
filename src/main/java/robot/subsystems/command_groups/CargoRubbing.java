package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.wrist_control.commands.GripperControl;
import robot.subsystems.wrist_control.commands.WristTurn;

/**
 *
 */
public class CargoRubbing extends CommandGroup {

    public CargoRubbing(int state, boolean isBackward) {

        robot.subsystems.wrist_control.Constants.GRIPPER_SPEED speed = getSpeed(state, isBackward);

        addParallel(new GripperControl(speed), 0.5);
        addSequential(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));

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
}

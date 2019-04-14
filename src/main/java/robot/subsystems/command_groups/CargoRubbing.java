package robot.subsystems.command_groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;
import robot.subsystems.wrist_control.Constants;
import robot.subsystems.wrist_control.commands.GripperControl;
import robot.subsystems.wrist_control.commands.WristTurn;

/**
 * Shoot the cargo a short distance whilst also turning the wrist, similarly to the movement team 179 does in their robot reveal
 */
public class CargoRubbing extends CommandGroup {

    /**
     * Command group. shoots the cargo and moves the wrist down
     * @param relative should the angle the wrist turns be relative, or absolute.
     */
    public CargoRubbing(boolean relative) {
        addParallel(new GripperControl(Constants.GRIPPER_SPEED.RUBBING), 0.5);
        if(!relative)
            addSequential(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));
        else
            addSequential(new WristTurn(Robot.wristControl.getWristAngle() + Constants.WRIST_ANGLES.RELATIVE_RUBBING.getValue()));

    }
}

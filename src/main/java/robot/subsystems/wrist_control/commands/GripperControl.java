package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.Robot;
import robot.subsystems.wrist_control.Constants;
import robot.subsystems.wrist_control.Constants.GRIPPER_SPEED;
import robot.subsystems.wrist_control.GripperWheels;

/**
 * Instant Command in charge of controlling the speed of the wrist wheels, the wheels in charge of grabbing and releasing cargo.
 *
 * @author Lior
 */
public class GripperControl extends CommandBase {
    private GripperWheels gripperWheels;
    private double speed;//speed of the gripper
    private double timeout = 0;
    private boolean useTrigger;
    private GenericHID.Hand hand;

    public GripperControl(GripperWheels gripperWheels, double speed) {
        addRequirements(gripperWheels);
        this.gripperWheels = gripperWheels;
        this.speed = speed;
        this.useTrigger = false;
    }

    public GripperControl(GripperWheels gripperWheels, double speed, boolean useTrigger, GenericHID.Hand wantedHand) {
        addRequirements(gripperWheels);
        this.gripperWheels = gripperWheels;
        this.useTrigger = useTrigger;
        this.speed = speed;
        this.hand = wantedHand;
    }

    public GripperControl(GripperWheels gripperWheels, GRIPPER_SPEED gripperSpeed, boolean useTrigger, GenericHID.Hand wantedHand) {
        this(gripperWheels, gripperSpeed.getValue(), useTrigger, wantedHand);
    }

    public GripperControl(GripperWheels gripperWheels, GRIPPER_SPEED gripperSpeed) {
        this(gripperWheels, gripperSpeed.getValue());
    }

    public GripperControl(GripperWheels gripperWheels, double speed, double timeout) {
        this(gripperWheels, speed);
        this.timeout = timeout;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (!(gripperWheels.isCargoInside() && speed < 0)) {
            if (useTrigger)
                gripperWheels.setGripperSpeed(speed * (Constants.TRIGGER_MINIMAL_VALUE + (1 - Constants.TRIGGER_MINIMAL_VALUE) * Robot.m_oi.xbox.getTriggerAxis(hand))); //scale the trigger with its value, starting from the constant starting value, with the maximum speed being the parameter
            else
                gripperWheels.setGripperSpeed(speed);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        if (speed < 0)
            return gripperWheels.isCargoInside() && speed < 0;
        else {
            return false;
        }
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        if (speed < 0)
            gripperWheels.setGripperSpeed(-0.1);
        else
            gripperWheels.setGripperSpeed(0);
    }
}
package robot.subsystems.wrist_control.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import robot.OI;
import robot.Robot;
import robot.subsystems.wrist_control.Constants;
import robot.subsystems.wrist_control.Constants.GRIPPER_SPEED;

import static robot.Robot.*;

/**
 * Instant Command in charge of controlling the speed of the wrist wheels, the wheels in charge of grabbing and releasing cargo.
 *
 * @author Lior
 */
public class GripperControl extends Command {
    private double speed;//speed of the gripper
    private double timeout = 0;
    private boolean useTrigger;
    private GenericHID.Hand hand;
    public GripperControl(double speed){
        requires(gripperWheels);
        this.speed = speed;
        this.useTrigger = false;
    }

    public GripperControl(double speed, boolean useTrigger, GenericHID.Hand wantedHand){
        requires(gripperWheels);
        this.useTrigger = useTrigger;
        this.speed = speed;
        this.hand = wantedHand;
    }

    public GripperControl(GRIPPER_SPEED gripperSpeed, boolean useTrigger, GenericHID.Hand wantedHand) {
        this(gripperSpeed.getValue(), useTrigger, wantedHand);
    }

    public GripperControl(GRIPPER_SPEED gripperSpeed) {
        this(gripperSpeed.getValue());
    }

    public GripperControl(double speed, double timeout) {
        this(speed);
        this.timeout = timeout;
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if(!climb.isClimbing()) {

            if (!(gripperWheels.isCargoInside() && speed < 0)) {
                if (useTrigger)
                    gripperWheels.setGripperSpeed(speed * (Constants.TRIGGER_MINIMAL_VALUE + (1 - Constants.TRIGGER_MINIMAL_VALUE) * OI.xbox.getTriggerAxis(hand))); //scale the trigger with its value, starting from the constant starting value, with the maximum speed being the parameter
                else
                    gripperWheels.setGripperSpeed(speed);
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(climb.isClimbing())
            return true;
        if (speed < 0)
            return gripperWheels.isCargoInside() && speed < 0;
        else {
            return false;
        }
    }

    // Called once after isFinished returns true
    protected void end()
    {
        if(speed < 0)
            gripperWheels.setGripperSpeed(-0.1);
        else
            gripperWheels.setGripperSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
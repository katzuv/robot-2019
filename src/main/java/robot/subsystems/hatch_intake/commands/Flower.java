package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static robot.Robot.hatchIntake;

/*
this command controls the flower on the robot
 */
public class Flower extends InstantCommand {

    private gripperState current;//enum variable that indicates the current mode of the flower

    /**
     * empty constructor, sets the wanted state to toggle meaning whenever the command is called it will toggle the current state
     * instead of going to the wanted state
     */
    public Flower() {
        addRequirements(hatchIntake);
        current = gripperState.TOGGLE_FLOWER;

    }

    /**
     * constructor that sets the wanted state
     *
     * @param open if true changes the wanted state to open and otherwise sets the wanted state to cloes
     */
    public Flower(boolean open) {
        addRequirements(hatchIntake);
        if (open)
            current = gripperState.FLOWER_GRAB;
        else
            current = gripperState.FLOWER_RELEASE;
    }

    @Override
    public void initialize() {
        switch (current) {
            case TOGGLE_FLOWER: // Change to the second state
                hatchIntake.setFlower(!hatchIntake.isFlowerOpen());
                break;
            case FLOWER_GRAB: // Open the gripper if closed and not do anything otherwise
                hatchIntake.setFlower(true);
                break;
            case FLOWER_RELEASE:// Close the gripper if opened and not do anything otherwise
                hatchIntake.setFlower(false);
                break;
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }


    /**
     * enum to indicate the state of the gripper
     */
    public enum gripperState {
        TOGGLE_FLOWER,
        FLOWER_GRAB,
        FLOWER_RELEASE
    }
}

package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import robot.subsystems.elevator.Elevator;

public class ResetElevatorHeight extends InstantCommand {

    private Elevator elevator;

    public ResetElevatorHeight(Elevator elevator) {
        addRequirements(elevator);
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.resetEncoders();
    }

}

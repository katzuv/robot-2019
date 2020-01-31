package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import robot.Robot;

public class ResetElevatorHeight extends InstantCommand {
    

    public void initialize() {
        Robot.elevator.resetEncoders();
    }

}

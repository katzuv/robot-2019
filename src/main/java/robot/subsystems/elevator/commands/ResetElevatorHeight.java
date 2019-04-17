package robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class ResetElevatorHeight extends InstantCommand {
    

    protected void initialize() {
        Robot.elevator.resetEncoders();
    }

}

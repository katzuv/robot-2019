package robot.subsystems.hatch_intake.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import robot.Robot;

public class HatchTransportation extends InstantCommand {
    private boolean down;
    public HatchTransportation(boolean down) {
        this.down=down;
        requires(Robot.hatchIntake);
    }

    @Override
    public void initialize() {
        Robot.hatchIntake.setGroundIntake(down);
    }

    @Override
    public void execute() {
    }


    @Override
    protected void end() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}

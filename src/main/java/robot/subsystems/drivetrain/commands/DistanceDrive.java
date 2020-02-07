package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import robot.subsystems.drivetrain.Constants;
import robot.subsystems.drivetrain.Drivetrain;


/**
 * This command drives a specified distance using motion magic.
 */
public class DistanceDrive extends CommandBase {
    private double distance;
    private double startDistance;
    private Drivetrain drivetrain;

    public DistanceDrive(Drivetrain drivetrain, double distance) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        startDistance = (drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2;
        drivetrain.driveDistance(distance, distance); //TODO: currently this doesn't use the real values
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return Math.abs((distance + startDistance) - ((drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2)) < Constants.ENDING_TOLERANCE;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        drivetrain.setSpeed(0, 0);
    }

}
package robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import robot.Robot;

/**
 * Set the location of the Drivetrain.
 */
public class SetLocation extends InstantCommand {

    private final Pose2d location;

    public SetLocation(Pose2d location) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.location = location;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.drivetrain.resetLocation(location);
    }

}
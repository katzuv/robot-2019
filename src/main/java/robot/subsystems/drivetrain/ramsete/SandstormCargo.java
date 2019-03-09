package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.Robot;

/**
 *
 */
public class SandstormCargo extends CommandGroup {

    public SandstormCargo() {
        Pose2d center = new Pose2d(LengthKt.getFeet(5.194), LengthKt.getFeet(13.587), Rotation2dKt.getDegree(180));
        addSequential(new DrivePathVision(center, new Pose2d(LengthKt.getFeet(16.28), LengthKt.getFeet(12.607), Rotation2dKt.getDegree(180)), true));
        addSequential(new DrivePathVision(Robot.drivetrain.getRobotPosition().transformBy(new Pose2d(Length.Companion.getKZero(), Length.Companion.getKZero(), Rotation2dKt.getDegree(-180))), new Pose2d(LengthKt.getFeet(10.119), LengthKt.getFeet(2.383), Rotation2dKt.getDegree(180)), false));
    }
}
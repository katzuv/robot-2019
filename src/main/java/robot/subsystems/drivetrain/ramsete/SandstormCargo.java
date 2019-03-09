package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.Robot;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class SandstormCargo extends CommandGroup {

    public SandstormCargo() {
        List<Pose2d> toCargoShip = new ArrayList<>();
        toCargoShip.add(new Pose2d(LengthKt.getFeet(5.194), LengthKt.getFeet(13.587), Rotation2dKt.getDegree(180)));
        toCargoShip.add(new Pose2d(LengthKt.getFeet(16.28), LengthKt.getFeet(12.607), Rotation2dKt.getDegree(180)));
        addSequential(new DrivePathVision(toCargoShip, true));
        List<Pose2d> toLoadingStation = new ArrayList<>();
        toLoadingStation.add(Robot.drivetrain.getRobotPosition().transformBy(new Pose2d(Length.Companion.getKZero(), Length.Companion.getKZero(), Rotation2dKt.getDegree(-180))));
        toLoadingStation.add(new Pose2d(LengthKt.getFeet(9.875), LengthKt.getFeet(18.682), Rotation2dKt.getDegree(110)));
        toLoadingStation.add(new Pose2d(LengthKt.getFeet(10.119), LengthKt.getFeet(24.883), Rotation2dKt.getDegree(180)));
        addSequential(new DrivePathVision(toLoadingStation, false));
    }
}

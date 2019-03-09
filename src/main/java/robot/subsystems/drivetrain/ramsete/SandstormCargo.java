package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class SandstormCargo extends CommandGroup {

    public SandstormCargo() {
        List<Pose2d> toCargoShip = new ArrayList<>();
        toCargoShip.add(new Pose2d(LengthKt.getFeet(16.28), LengthKt.getFeet(12.607), Rotation2dKt.getDegree(180)));
        addSequential(new DrivePathVision(toCargoShip, true, true, 0, 0));
        List<Pose2d> toLoadingStationFront = new ArrayList<>();
        toLoadingStationFront.add(new Pose2d(LengthKt.getFeet(11.43), LengthKt.getFeet(2.125), Rotation2dKt.getDegree(0)));
        addSequential(new DrivePathVision(toLoadingStationFront, false, false, 0, 0.8));
        List<Pose2d> toLoadingStation = new ArrayList<>();
        toLoadingStation.add(new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(2.216), Rotation2dKt.getDegree(180)));
        addSequential(new DrivePathVision(toLoadingStation, true, true, 0.8, 0));
    }
}

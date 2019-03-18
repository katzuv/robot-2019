package robot.subsystems.drivetrain.ramsete;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class SandstormRocket extends CommandGroup {

    public SandstormRocket(Constants.ELEVATOR_STATES height) {
        List<Pose2d> toRocket = new ArrayList<>();
        toRocket.add(new Pose2d(LengthKt.getFeet(22.584), LengthKt.getFeet(2.119), Rotation2dKt.getDegree(30)));
        addParallel(new ElevatorCommand(height));
        addSequential(new DrivePathVision(toRocket, true, true, 0, 0, false));

//        addSequential(new HatchScoring(height, false));

//        List<Pose2d> toLoadingStationFront = new ArrayList<>();
//        toLoadingStationFront.add(new Pose2d(LengthKt.getFeet(12.978), LengthKt.getFeet(3.982), Rotation2dKt.getDegree(90)));
//        addSequential(new DrivePathVision(toLoadingStationFront, false, false, 0, 0, false));

//        List<Pose2d> toLoadingStation = new ArrayList<>();
//        toLoadingStation.add(new Pose2d(LengthKt.getFeet(5), LengthKt.getFeet(2.216), Rotation2dKt.getDegree(180)));
//        addSequential(new DrivePathVision(toLoadingStation, true, true, 0, 0));
//
//        List<Pose2d> toBackRocket = new ArrayList<>();
//        toBackRocket.add(new Pose2d(LengthKt.getFeet(19.107), LengthKt.getFeet(4.724), Rotation2dKt.getDegree(0)));
//        toBackRocket.add(new Pose2d(LengthKt.getFeet(23.592), LengthKt.getFeet(3.444), Rotation2dKt.getDegree(30)));
//        addSequential(new DrivePathVision(toBackRocket, false, false, 0, 0));
//
//        List<Pose2d> placeFinalHatch = new ArrayList<>();
//        placeFinalHatch.add(new Pose2d(LengthKt.getFeet(21.877), LengthKt.getFeet(2.48), Rotation2dKt.getDegree(30)));
//        addSequential(new DrivePathVision(placeFinalHatch, true, true, 0, 0));
    }
}
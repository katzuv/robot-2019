package robot.subsystems.drivetrain.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.subsystems.drivetrain.commands.DistanceDrive;
import robot.subsystems.drivetrain.commands.SetLocation;
import robot.subsystems.drivetrain.commands.TurnAngle;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.drivetrain.ramsete.TrajectoryTracker;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Flower;

import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous command group for the sandstorm period.
 * puts a hatch in the left rocket
 * takes another from the loading station
 * put a second hatch on the far side of the left rocket.
 */
public class TwoHatchLeftRocket extends CommandGroup {

    public TwoHatchLeftRocket(Constants.ELEVATOR_HEIGHTS height) {
        addSequential(new SetLocation(new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(17.645), Rotation2dKt.getDegree(180))));

        addParallel(new ElevatorCommand(height));

        TrajectoryTracker toRocketCommand = new TrajectoryTracker(Paths.LEFT_HAB_TO_NEAR_ROCKET, false);
        addSequential(toRocketCommand);

//        addParallel(new Fangs(true));

        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        //Score hatch
        //addSequential(new PlaceHatch(height));
        addSequential(new WaitCommand(0.5));

        //addParallel(new RetractHatch());
        addSequential(new WaitCommand(0.5));

        List<Pose2d> toLoadingStation = new ArrayList<>();
        toLoadingStation.add(new Pose2d(LengthKt.getFeet(9.463), LengthKt.getFeet(23.7), Rotation2dKt.getDegree(180)));
        addSequential(new TrajectoryTracker(toLoadingStation, 0, 1, false, false));

        addSequential(new TurnAngle(185));

        addSequential(new WaitCommand(0.4));

//        addSequential(new Fangs(true));
        addSequential(new Flower(true));
        addSequential(new VisionDrive());

        addSequential(new WaitCommand(0.2));

//        addSequential(new TakeHatch());

        addSequential(new DistanceDrive(0.5));

//        addSequential(new WaitCommand(0.3));
        List<Pose2d> driveWithHatch = new ArrayList<>();
        driveWithHatch.add(new Pose2d(LengthKt.getFeet(19.015), LengthKt.getFeet(21), Rotation2dKt.getDegree(0)));
        driveWithHatch.add(new Pose2d(LengthKt.getFeet(25.563), LengthKt.getFeet(23.397), Rotation2dKt.getDegree(30)));

        addSequential(new TrajectoryTracker(driveWithHatch, 0, 0, false, false));

//        addSequential(new WaitCommand(0.5));
//        addParallel(new Fangs(true));
//        addSequential(new VisionDrive());
//        addSequential(new WaitCommand(0.2));
//
//        addSequential(new HatchScoring(height));
    }
}
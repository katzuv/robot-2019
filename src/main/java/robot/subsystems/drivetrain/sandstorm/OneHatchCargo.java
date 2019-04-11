package robot.subsystems.drivetrain.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.subsystems.command_groups.HatchScoring;
import robot.subsystems.command_groups.TakeHatch;
import robot.subsystems.drivetrain.commands.DistanceDrive;
import robot.subsystems.drivetrain.commands.SetLocation;
import robot.subsystems.drivetrain.commands.TurnAngle;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.drivetrain.ramsete.DrivePathVision;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.ExtensionPlate;
import robot.subsystems.hatch_intake.commands.Flower;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class OneHatchCargo extends CommandGroup {

    public OneHatchCargo(Constants.ELEVATOR_STATES height) {
        addSequential(new SetLocation(new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(13.565), Rotation2dKt.getDegree(180))));

        addParallel(new ElevatorCommand(height));

        addSequential(new DrivePathVision(Paths.MIDDLE_HAB_TO_RIGHT_CARGO, false));

        addSequential(new WaitCommand(0.2));

        addParallel(new ExtensionPlate(true));
        addSequential(new VisionDrive());

        addSequential(new HatchScoring(height));

        addSequential(new WaitCommand(0.3));

        List<Pose2d> toLoadingStation = new ArrayList<>();
        toLoadingStation.add(new Pose2d(LengthKt.getFeet(9.463), LengthKt.getFeet(23.7), Rotation2dKt.getDegree(180)));
        addSequential(new DrivePathVision(toLoadingStation, 1, 1, 3, 2.5, false, false));
        addSequential(new WaitCommand(0.2));

        addSequential(new TurnAngle(185));

        addSequential(new WaitCommand(0.5));

        addSequential(new VisionDrive());
        addParallel(new ExtensionPlate(true));
        addParallel(new Flower(true));
        addSequential(new TakeHatch());

        addSequential(new DistanceDrive(0.5));
    }
}
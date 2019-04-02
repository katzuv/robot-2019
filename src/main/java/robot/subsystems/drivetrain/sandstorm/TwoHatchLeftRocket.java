package robot.subsystems.drivetrain.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.subsystems.command_groups.HatchScoring;
import robot.subsystems.command_groups.PlaceHatch;
import robot.subsystems.command_groups.RetractHatch;
import robot.subsystems.command_groups.TakeHatch;
import robot.subsystems.drivetrain.commands.DistanceDrive;
import robot.subsystems.drivetrain.commands.ResetLocation;
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
public class TwoHatchLeftRocket extends CommandGroup {

    public TwoHatchLeftRocket(Constants.ELEVATOR_STATES height) {
        addSequential(new ResetLocation(new Pose2d(LengthKt.getFeet(6.321), LengthKt.getFeet(17.645), Rotation2dKt.getDegree(180))));

        addParallel(new ElevatorCommand(height));

        DrivePathVision toRocketCommand = new DrivePathVision(Paths.LEFT_HAB_TO_NEAR_ROCKET, false);
        addSequential(toRocketCommand);

        addParallel(new ExtensionPlate(true));

        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        //Score hatch
        addSequential(new PlaceHatch(height));
        addSequential(new WaitCommand(0.5));

        addParallel(new RetractHatch());
        addSequential(new WaitCommand(0.5));

        List<Pose2d> toLoadingStation = new ArrayList<>();
        toLoadingStation.add(new Pose2d(LengthKt.getFeet(7.769), LengthKt.getFeet(23.954), Rotation2dKt.getDegree(180)));
        addSequential(new DrivePathVision(toLoadingStation, 0, 1, false, false));

        addSequential(new TurnAngle(175));

        addSequential(new WaitCommand(0.4));

        addSequential(new ExtensionPlate(true));
        addSequential(new Flower(true));
        addSequential(new VisionDrive());

        addSequential(new WaitCommand(0.2));

        addSequential(new TakeHatch());

        addSequential(new DistanceDrive(0.5));

        addSequential(new WaitCommand(0.3));
        List<Pose2d> driveWithHatch = new ArrayList<>();
        driveWithHatch.add(new Pose2d(LengthKt.getFeet(19.015), LengthKt.getFeet(21), Rotation2dKt.getDegree(0)));
        driveWithHatch.add(new Pose2d(LengthKt.getFeet(25.563), LengthKt.getFeet(23.397), Rotation2dKt.getDegree(30)));

        addSequential(new DrivePathVision(driveWithHatch, 0, 0, false, false));

        addSequential(new WaitCommand(0.5));
        addParallel(new ExtensionPlate(true));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        addSequential(new HatchScoring(height));
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
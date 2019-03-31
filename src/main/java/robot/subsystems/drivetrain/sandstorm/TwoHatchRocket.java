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
public class TwoHatchRocket extends CommandGroup {

    public TwoHatchRocket(Constants.ELEVATOR_STATES height) {
        //Lift elevator before time
        addParallel(new ElevatorCommand(height));

        DrivePathVision toRocketCommand = new DrivePathVision(Paths.RIGHT_HAB_TO_NEAR_ROCKET, false);
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
        toLoadingStation.add(new Pose2d(LengthKt.getFeet(7.769), LengthKt.getFeet(2.187), Rotation2dKt.getDegree(180)));
        addSequential(new DrivePathVision(toLoadingStation, 0, 1, false, false));

        addSequential(new TurnAngle(140));

        addSequential(new WaitCommand(0.5));

        addSequential(new ExtensionPlate(true));
        addSequential(new Flower(true));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        addSequential(new TakeHatch());

        addSequential(new DistanceDrive(0.5));

        addSequential(new TurnAngle(180));

        addSequential(new WaitCommand(0.3));

        //Drive to rocket
        List<Pose2d> toRocket2nd = new ArrayList<>();
        toRocket2nd.add(new Pose2d(LengthKt.getFeet(15.559), LengthKt.getFeet(3.011), Rotation2dKt.getDegree(150)));

        addSequential(new DrivePathVision(toRocket2nd, 0, 1, true, false));

        addParallel(new ExtensionPlate(true));

        addSequential(new WaitCommand(0.4));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        addSequential(new HatchScoring(height, false));
    }

}
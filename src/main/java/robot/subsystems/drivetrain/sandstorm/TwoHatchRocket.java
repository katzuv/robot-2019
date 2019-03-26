package robot.subsystems.drivetrain.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.subsystems.command_groups.HatchScoring;
import robot.subsystems.drivetrain.commands.DistanceDrive;
import robot.subsystems.drivetrain.commands.TurnAngle;
import robot.subsystems.drivetrain.commands.VisionDrive;
import robot.subsystems.drivetrain.ramsete.DrivePathVision;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.command_groups.TakeHatch;
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
        //Drive to rocket
        List<Pose2d> toRocket = new ArrayList<>();
        toRocket.add(new Pose2d(LengthKt.getFeet(11.68), LengthKt.getFeet(4.789), Rotation2dKt.getDegree(150)));
        addSequential(new DrivePathVision(toRocket, true, false, 1, 1.7, false));

        addParallel(new ExtensionPlate(true));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        //Score hatch
        addSequential(new HatchScoring(height, false));

        addSequential(new DistanceDrive(0.5));

        addSequential(new TurnAngle(90));

        addSequential(new WaitCommand(0.5));

        addSequential(new DistanceDrive(-0.5));

        addSequential(new ExtensionPlate(true));
        addSequential(new Flower(true));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        addSequential(new TakeHatch());

        addSequential(new WaitCommand(0.1));

        List<Pose2d> driveWithHatch = new ArrayList<>();
        driveWithHatch.add(new Pose2d(LengthKt.getFeet(12.552), LengthKt.getFeet(5.861), Rotation2dKt.getDegree(0)));
        driveWithHatch.add(new Pose2d(LengthKt.getFeet(19.751), LengthKt.getFeet(5.861), Rotation2dKt.getDegree(0)));
        driveWithHatch.add(new Pose2d(LengthKt.getFeet(25.504), LengthKt.getFeet(4.602), Rotation2dKt.getDegree(30)));
        addSequential(new DrivePathVision(driveWithHatch, false, false, 0, 1, false));

//        List<Pose2d> driveToSecondRocket = new ArrayList<>();

//        addSequential(new DrivePathVision(driveToSecondRocket, false, false, 1, 0, false));

        addParallel(new ExtensionPlate(true));
        addSequential(new WaitCommand(0.4));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        addSequential(new HatchScoring(height, false));
    }

}

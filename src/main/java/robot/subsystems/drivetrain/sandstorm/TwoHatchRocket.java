package robot.subsystems.drivetrain.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import robot.Robot;
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
import robot.utilities.Utils;

import java.util.ArrayList;
import java.util.List;

/**
 *
 */
public class TwoHatchRocket extends CommandGroup {

    public TwoHatchRocket(Constants.ELEVATOR_STATES height) {
        //Lift elevator before time
        //addParallel(new ElevatorCommand(height));
        //Drive to rocket
        addSequential(new DrivePathVision(Paths.RIGHT_HAB_TO_NEAR_ROCKET, false));
        //addParallel(new ExtensionPlate(true));
        addSequential(new VisionDrive());
        //addSequential(new WaitCommand(0.2));
        //Score hatch
        //addSequential(new PlaceHatch(height));
        addSequential(new WaitCommand(0.5));
        //addParallel(new RetractHatch());
        addSequential(new DistanceDrive(0.5));
        addSequential(new TurnAngle(110));
        addSequential(new WaitCommand(0.5));
        ////addSequential(new DrivePathVision(Paths.NEAR_ROCKET_TO_LOADING, false));

        addSequential(new DistanceDrive(-0.5));
        //addSequential(new ExtensionPlate(true));
        //addSequential(new Flower(true));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));
        //addSequential(new TakeHatch());
        addSequential(new WaitCommand(0.1));
        addSequential(new DrivePathVision(Paths.LOADING_TO_FAR_ROCKET, false));

        addSequential(new WaitCommand(0.1));
        addSequential(new TurnAngle(-120));
        //addParallel(new ExtensionPlate(true));
        addSequential(new WaitCommand(0.4));
        addSequential(new VisionDrive());
        addSequential(new WaitCommand(0.2));

        addSequential(new HatchScoring(height, false));
    }

}

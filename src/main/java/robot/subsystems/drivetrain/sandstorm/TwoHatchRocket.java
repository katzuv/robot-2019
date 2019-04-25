package robot.subsystems.drivetrain.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.command_groups.VisionPlaceHatch;
import robot.subsystems.command_groups.VisionTakeHatch;
import robot.subsystems.drivetrain.commands.DistanceDrive;
import robot.subsystems.drivetrain.commands.TurnAngle;
import robot.subsystems.drivetrain.talon_profiling.Profiles;
import robot.subsystems.drivetrain.talon_profiling.TalonFollow;
import robot.subsystems.elevator.Constants;
import robot.subsystems.elevator.commands.ElevatorCommand;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.wrist_control.commands.WristTurn;

/**
 *
 */
public class TwoHatchRocket extends CommandGroup {

    public TwoHatchRocket(boolean isRight) {
        addParallel(new Flower(false));
        addParallel(new Flower(false));
        addParallel(new ElevatorCommand(Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH));
        addParallel(new CommandGroup(){{
            addSequential(new WaitCommand(1));
            addSequential(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));
        }});
        addSequential(new TalonFollow(Profiles.toNearRocketLeft, Profiles.toNearRocketRight, !isRight));
        addSequential(new VisionPlaceHatch(Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH));
        addSequential(new DistanceDrive(-0.3));
        addSequential(new TurnAngle(174, true));
        addSequential(new DistanceDrive(4), 1.5); //its not what it looks like i swear
        addSequential(new VisionTakeHatch());
//        addSequential(new TalonFollow(Profiles.loadingStationToFarRocketLeft, Profiles.loadingStationToFarRocketRight));
//        addSequential(new VisionPlaceHatch(Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH));
    }
}
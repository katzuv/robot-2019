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

public class RightCargoShip extends CommandGroup {

    public RightCargoShip() {
        addParallel(new Flower(false));
        addParallel(new ElevatorCommand(Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH));
        addParallel(new CommandGroup(){{
            addSequential(new WaitCommand(1.5));
            addSequential(new WristTurn(robot.subsystems.wrist_control.Constants.WRIST_ANGLES.FORWARD));
        }});
        addSequential(new TalonFollow(Profiles.toRightCargoLeft, Profiles.toRightCargoRight, false));
        addSequential(new VisionPlaceHatch(Constants.ELEVATOR_HEIGHTS.LEVEL1_HATCH));
        addSequential(new TalonFollow(Profiles.rightCargoToLoadingLeft, Profiles.rightCargoToLoadingRight, false));
        addSequential(new TurnAngle(174, true));
        addSequential(new VisionTakeHatch());
        addSequential(new TalonFollow(Profiles.loadingStationToFarRocketLeft, Profiles.loadingStationToFarRocketRight));
    }

}

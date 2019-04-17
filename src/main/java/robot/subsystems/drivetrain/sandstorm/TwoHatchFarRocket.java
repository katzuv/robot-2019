package robot.subsystems.drivetrain.sandstorm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import robot.subsystems.command_groups.VisionPlaceHatch;
import robot.subsystems.drivetrain.talon_profiling.Profiles;
import robot.subsystems.drivetrain.talon_profiling.TalonFollow;
import robot.subsystems.elevator.Constants;
import robot.subsystems.hatch_intake.commands.Flower;
import robot.subsystems.wrist_control.commands.WristTurn;

/**
 *
 */
public class TwoHatchFarRocket extends CommandGroup {

    public TwoHatchFarRocket(Constants.ELEVATOR_HEIGHTS height , boolean isRight) {
            addSequential(new TalonFollow(Profiles.toFarRocketLeft, Profiles.toFarRocketRight, !isRight));
            addSequential(new VisionPlaceHatch(height));
//            addParallel(new CommandGroup() {{
//                addSequential(new WaitCommand(0.8));
//                addParallel(new Flower(false));
//                addSequential(new WristTurn(0));
//
//            }}); //Close wrist and flower while driving from the rocket to the loading station
            //addSequential(new TalonFollow(Profiles.farRocketToLoadingRevLeft, Profiles.farRocketToLoadingRevRight, !isRight));
    }
}